#pragma once
// Included by the command dispatcher. It is the single writer of the requested
// source. A producer may concurrently invalidate only its own frame.
#include "feedback_source.h"

// Publish the split 64-bit switch time before the atomic token, so a late
// pre-switch acquisition cannot establish the new frame. Cortex-M4 supports
// atomic 32-bit loads/stores; this avoids an RTOS lock in producer callbacks.
static uint32_t switch_high, switch_low;
static uint32_t selection = 4U | FEEDBACK_SOURCE_ENCODER;
static uint32_t selection_token(void) { return __atomic_load_n(&selection, __ATOMIC_ACQUIRE); }
uint8_t feedback_requested(void) { return selection_token() & 3U; }

void feedback_restore_preference(uint8_t source)
{
    uint64_t at=hw_clock_microseconds();
    __atomic_store_n(&switch_high,(uint32_t)(at >> 32),__ATOMIC_RELAXED);
    __atomic_store_n(&switch_low,(uint32_t)at,__ATOMIC_RELAXED);
    uint32_t token = selection_token();
    do {
        uint32_t next = ((token + 4U) & ~3U) | source;
        if (__atomic_compare_exchange_n(&selection, &token, next, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
            return;
    } while (true);
}

void feedback_invalidate(uint8_t source)
{
    uint32_t token = selection_token();
    while ((token & 3U) == source &&
           !__atomic_compare_exchange_n(&selection, &token, token + 4U, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {}
}

void feedback_anchor(feedback_frame_t *frame, uint8_t source, uint64_t stamp, platform_odometry_t pose)
{
    uint32_t token = selection_token();
    if ((token & 3U) != source || token == frame->token) return;
    uint64_t after = ((uint64_t)__atomic_load_n(&switch_high,__ATOMIC_RELAXED) << 32) |
                     __atomic_load_n(&switch_low,__ATOMIC_RELAXED);
    if (stamp <= after) return;
    frame->anchor = pose;
    frame->token = token;
}

void feedback_snapshot(feedback_snapshot_t *out, const feedback_frame_t *frame, uint8_t source,
                       uint32_t sequence, uint64_t stamp, platform_odometry_t pose, platform_odometry_t twist)
{
    uint32_t token = selection_token();
    if (token != frame->token || (token & 3U) != source) {
        out->reason = FEEDBACK_REASON_WAITING_SAMPLE;
        return;
    }
    double dx = pose.x - frame->anchor.x, dy = pose.y - frame->anchor.y;
    double c = cos(frame->anchor.t), s = sin(frame->anchor.t);
    *out = (feedback_snapshot_t){.token=token, .sequence=sequence, .stamp=stamp,
        .pose={c*dx + s*dy, -s*dx + c*dy, pose.t - frame->anchor.t}, .twist=twist};
}

static bool feedback_switch_safe(void)
{
    if (platform_is_controller_running() || !hw_motor_outputs_stopped()) return false;
    for (uint8_t i=0; i<NUMBER_MOTORS; ++i)
        if (controllers_manager_is_running(i)) return false;
    return !tracking_reports_motion() && odometry_feedback_at_rest(hw_clock_microseconds());
}

static uint8_t feedback_select(uint8_t source)
{
    if (source > FEEDBACK_SOURCE_FUSED) return RESPONSE_INVALID_ARGUMENT;
    if (feedback_requested() == source) return RESPONSE_OK;
    if (!feedback_switch_safe()) return RESPONSE_SOURCE_SWITCH_UNSAFE;
    uint8_t error = passive_fusion_select(source == FEEDBACK_SOURCE_FUSED);
    if (error == RESPONSE_OK) feedback_restore_preference(source);
    return error;
}

// The serialized command path owns the ring. Natural source publication does
// not evict a keyed pair unless another status capture actually needs its slot.
static struct {
    feedback_snapshot_t snapshot;
    uint64_t captured_us;
    uint16_t max_age_ms;
} captures[8];
static unsigned next_capture;

static uint16_t feedback_max_age_ms(uint8_t source)
{
    if (source == FEEDBACK_SOURCE_ENCODER) return (uint16_t)(odometry_manager_get_period_ms()*3U);
    return source == FEEDBACK_SOURCE_IMU_ONLY ? 50 : 60;
}

static int find_capture(uint32_t token, uint32_t sequence)
{
    for (unsigned i=0; i<sizeof(captures)/sizeof(captures[0]); ++i)
        if (captures[i].snapshot.stamp && captures[i].snapshot.token == token &&
            captures[i].snapshot.sequence == sequence) return (int)i;
    return -1;
}

static void capture_snapshot(const feedback_snapshot_t *s,uint64_t now,uint16_t max_age_ms)
{
    int i=find_capture(s->token,s->sequence);
    if (i<0) { i=(int)next_capture; next_capture=(next_capture+1U)%8U; }
    captures[i].snapshot=*s;
    captures[i].captured_us=now;
    captures[i].max_age_ms=max_age_ms;
}

static uint8_t feedback_read(uint8_t command, uint8_t kind, uint32_t origin, uint32_t sequence,
                             void (*reply)(uint8_t *,uint8_t))
{
    if (kind > FEEDBACK_VECTOR_TWIST) return RESPONSE_INVALID_ARGUMENT;
    uint8_t source = feedback_requested();
    feedback_snapshot_t s = {.reason=FEEDBACK_REASON_WAITING_SAMPLE};
    switch (source) {
    case FEEDBACK_SOURCE_ENCODER: odometry_feedback_sample(&s); break;
    case FEEDBACK_SOURCE_IMU_ONLY: tracking_feedback_sample(&s); break;
    case FEEDBACK_SOURCE_FUSED: passive_fusion_sample(&s); break;
    }
    uint32_t token = selection_token();
    if (!s.reason && token != s.token) s.reason = FEEDBACK_REASON_WAITING_SAMPLE;
    uint64_t now=hw_clock_microseconds();
    uint16_t max_age_ms=feedback_max_age_ms(source);
    if (!s.reason && tracking_age(now,s.stamp) > (uint32_t)max_age_ms*1000U)
        s.reason=FEEDBACK_REASON_STALE;
    if (command == GET_FEEDBACK_SOURCE_STATUS) {
        bool valid = s.reason == FEEDBACK_REASON_OK;
        if (valid) capture_snapshot(&s,now,max_age_ms);
        feedback_source_status wire = {
            source, valid ? source : FEEDBACK_SOURCE_NONE, valid, s.reason,
            valid ? s.sequence : 0, valid ? s.stamp : 0, token >> 2, s.detail,
            valid ? tracking_age(now, s.stamp) : UINT32_MAX, max_age_ms
        };
        reply((uint8_t *)&wire,sizeof(wire));
    } else {
        if (s.reason || (token >> 2) != origin) return RESPONSE_SAMPLE_NOT_AVAILABLE;
        int i=find_capture(token,sequence);
        if (i<0 || now < captures[i].captured_us || now-captures[i].captured_us>50000U)
            return RESPONSE_SAMPLE_NOT_AVAILABLE;
        s=captures[i].snapshot;
        if (captures[i].max_age_ms < max_age_ms) max_age_ms=captures[i].max_age_ms;
        if (tracking_age(now,s.stamp) > (uint32_t)max_age_ms*1000U) return RESPONSE_SAMPLE_NOT_AVAILABLE;
        platform_odometry_t v = kind == FEEDBACK_VECTOR_POSE ? s.pose : s.twist;
        feedback_source_vector wire = {s.sequence,v.x,v.y,v.t};
        reply((uint8_t *)&wire,sizeof(wire));
    }
    return RESPONSE_OK;
}
