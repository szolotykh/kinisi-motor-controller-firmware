#include "config_store.h"
#include "board_revision.h"

#ifdef MP_V4
#include <stddef.h>
#include <string.h>
#include "hw_fram.h"

// Slots start on 256-byte boundaries. The independent one-byte commit marker
// lives at the end of each slot so a partial body write cannot touch it.
enum { SLOT_SIZE = 256, MARKER_OFFSET = 255, COMMITTED = 0xc3 };
typedef struct {
    uint32_t magic;
    uint32_t generation;
    uint16_t version;
    uint16_t length;
    uint8_t model, major, minor, patch;
    uint8_t payload[CONFIG_STORE_MAX_PAYLOAD];
    uint32_t crc;
} record_t;

_Static_assert(sizeof(record_t) == 212 && sizeof(record_t) <= MARKER_OFFSET, "FRAM record layout");
// Storage work is serialized by the command manager. Keep buffers off its stack.
static record_t candidate, readback;
static const uint32_t magic = 0x49534e4bU;

static uint32_t checksum(const record_t *record)
{
    uint32_t crc = UINT32_MAX;
    const uint8_t *bytes = (const uint8_t *)record;
    for (size_t i = 0; i < offsetof(record_t, crc); ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
    }
    return ~crc;
}

// Distinguish invalid records from a failed bus transaction. An unreadable
// slot might contain the newest setup and must never be selected for overwrite.
static bool find_current(uint16_t length, int *current, uint32_t *generation)
{
    *current = -1;
    *generation = 0;
    for (int slot = 0; slot < 2; ++slot) {
        const uint16_t base = (uint16_t)(slot * SLOT_SIZE);
        uint8_t marker;
        if (!hw_fram_read(base + MARKER_OFFSET, &marker, 1)) return false;
        if (marker != COMMITTED) continue;
        if (!hw_fram_read(base, &readback, sizeof(readback))) return false;
        if (readback.magic != magic || readback.version != 1 || readback.length != length ||
            readback.model != KINISI_BOARD_MODEL || readback.major != KINISI_BOARD_VERSION_MAJOR ||
            readback.minor != KINISI_BOARD_VERSION_MINOR || readback.patch != KINISI_BOARD_VERSION_PATCH ||
            readback.crc != checksum(&readback)) continue;
        const uint32_t distance = readback.generation - *generation;
        if (*current < 0 || (distance != 0 && distance < 0x80000000U)) {
            *current = slot;
            *generation = readback.generation;
            candidate = readback;
        }
    }
    return true;
}

bool config_store_load(void *data, uint16_t length)
{
    int current;
    uint32_t generation;
    if (!data || !length || length > CONFIG_STORE_MAX_PAYLOAD ||
        !find_current(length, &current, &generation) || current < 0) return false;
    memcpy(data, candidate.payload, length);
    return true;
}

static bool set_marker(uint16_t base, uint8_t value)
{
    uint8_t actual;
    return hw_fram_write(base + MARKER_OFFSET, &value, 1) &&
           hw_fram_read(base + MARKER_OFFSET, &actual, 1) && actual == value;
}

bool config_store_save(const void *data, uint16_t length)
{
    int current;
    uint32_t generation;
    if (!data || !length || length > CONFIG_STORE_MAX_PAYLOAD ||
        !find_current(length, &current, &generation)) return false;
    const uint16_t target = current == 0 ? SLOT_SIZE : 0;
    memset(&candidate, 0, sizeof(candidate));
    candidate.magic = magic;
    candidate.generation = generation + 1U;
    candidate.version = 1;
    candidate.length = length;
    candidate.model = KINISI_BOARD_MODEL;
    candidate.major = KINISI_BOARD_VERSION_MAJOR;
    candidate.minor = KINISI_BOARD_VERSION_MINOR;
    candidate.patch = KINISI_BOARD_VERSION_PATCH;
    memcpy(candidate.payload, data, length);
    candidate.crc = checksum(&candidate);
    return set_marker(target, 0) &&
           hw_fram_write(target, &candidate, sizeof(candidate)) &&
           hw_fram_read(target, &readback, sizeof(readback)) &&
           memcmp(&candidate, &readback, sizeof(candidate)) == 0 &&
           set_marker(target, COMMITTED);
}
#else
bool config_store_load(void *data, uint16_t length) { (void)data; (void)length; return false; }
bool config_store_save(const void *data, uint16_t length) { (void)data; (void)length; return false; }
#endif
