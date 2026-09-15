//------------------------------------------------------------
// File name: test_controller_stop.c
// Description: Exercise motor stop/brake before and after PID task creation.
//------------------------------------------------------------
#include "controllers_manager.h"
#include "cmsis_os.h"
#include "semphr.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

static unsigned locked, created, delays;
static unsigned stopped[4], braked[4], driven[4];
static void (*task_fn)(void *);
static void *task_arg;
static jmp_buf task_exit;

/** @brief Reject the NULL mutex that previously halted open-loop platform stops. */
int xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout)
{
    (void)timeout; assert(mutex == &locked && !locked); locked = 1; return 1;
}
/** @brief Check that every acquired lock is released. */
void xSemaphoreGive(SemaphoreHandle_t mutex) { assert(mutex == &locked && locked); locked = 0; }
/** @brief Track whether stopping an open-loop motor unnecessarily starts PID. */
SemaphoreHandle_t xSemaphoreCreateMutex(void) { created++; return &locked; }
/** @brief Treat a missing task as an uninitialized manager. */
osThreadState_t osThreadGetState(osThreadId_t thread) { return thread ? 0 : osThreadError; }
/** @brief Capture the real PID task for deterministic execution. */
osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr)
{
    (void)attr; task_fn = fn; task_arg = arg; return arg;
}
/** @brief Supply the initial task tick. */
TickType_t xTaskGetTickCount(void) { return 0; }
/** @brief Exit after one complete PID update and verify the lock was released. */
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    (void)last; (void)period; assert(!locked);
    if (delays++) longjmp(task_exit, 1);
}
/** @brief Record which physical motor receives a coast command. */
static void stop(motorIndex index) { assert(index < 4); stopped[index]++; }
/** @brief Record which physical motor receives a brake command. */
static void brake(motorIndex index) { assert(index < 4); braked[index]++; }
/** @brief Detect PID writes that could override a completed stop. */
static void speed(motorIndex index, double value) { (void)value; driven[index]++; }
/** @brief Supply harmless, stationary encoder feedback. */
static uint16_t count(encoder_index_t index) { (void)index; return 0; }
/** @brief Supply a nonzero encoder resolution. */
static double resolution(encoder_index_t index) { (void)index; return 100; }
/** @brief Expose motor callbacks without initializing a PID controller. */
const hw_motor_interface_t *get_motor_interface(void)
{
    static const hw_motor_interface_t hardware = {.stop = stop, .brake = brake, .set_speed = speed};
    return &hardware;
}
/** @brief Expose encoder callbacks for the real PID task. */
const hw_encoder_interface_t *get_encoder_interface(void)
{
    static const hw_encoder_interface_t hardware = {.get_value = count, .get_resolution = resolution};
    return &hardware;
}
/** @brief Execute a single PID iteration after changing controller state. */
static void tick(void)
{
    delays = 0;
    if (!setjmp(task_exit)) task_fn(task_arg);
}
/** @brief Verify open-loop, mixed ownership, repeated stops, and no PID restart. */
int main(void)
{
    // Single-motor STOP/BRAKE first disable PID, then apply the hardware action
    // in command_handler.c. Disabling PID must be safe before any task exists.
    for (uint8_t index = 0; index < 4; index++) {
        controllers_manager_stop_controller(index);
        assert(!controllers_manager_is_running(index));
        assert(!stopped[index] && !braked[index] && !driven[index]);
    }
    assert(!created && !locked);

    // No PID has ever been started: nonzero motor indexes must still stop.
    controllers_manager_brake_multiple(BMOTOR1 | BMOTOR3);
    controllers_manager_stop_controller_multiple(BMOTOR1 | BMOTOR3);
    assert(!created && !locked);
    assert(braked[0] == 0 && braked[1] == 1 && braked[2] == 0 && braked[3] == 1);
    assert(stopped[0] == 0 && stopped[1] == 1 && stopped[2] == 0 && stopped[3] == 1);

    // Motor 2 has PID; motor 3 never did. Both actions must use the mask index.
    controllers_manager_initialize_controller_multiple(BMOTOR2, 1, 0, 0, 10);
    controllers_manager_set_target_speed(2, 1);
    tick();
    assert(driven[2] == 1 && created == 1);
    controllers_manager_brake_multiple(BMOTOR1 | BMOTOR3);
    assert(controllers_manager_is_running(2));
    controllers_manager_stop_controller_multiple(BMOTOR2 | BMOTOR3);
    assert(!controllers_manager_is_running(2));
    tick();
    assert(driven[2] == 1); // Stopped PID must not overwrite coast on its next tick.
    assert(stopped[0] == 0 && stopped[1] == 1 && stopped[2] == 1 && stopped[3] == 2);

    controllers_manager_initialize_controller_multiple(BMOTOR2, 1, 0, 0, 10);
    controllers_manager_brake_multiple(BMOTOR2);
    controllers_manager_brake_multiple(BMOTOR2);
    tick();
    assert(!controllers_manager_is_running(2) && !locked);
    assert(braked[2] == 2 && driven[2] == 1);

    // Exercise the single-motor PID stop path for every motor, with other PIDs
    // still running. Hardware output stays unchanged until the caller selects
    // brake or coast, and later PID ticks must never override that selection.
    controllers_manager_initialize_controller_multiple(0x0f, 1, 0, 0, 10);
    for (uint8_t index = 0; index < 4; index++) {
        unsigned stop_count = stopped[index], brake_count = braked[index];
        controllers_manager_stop_controller(index);
        controllers_manager_stop_controller(index); // Repeated stops are harmless.
        assert(!controllers_manager_is_running(index));
        assert(stopped[index] == stop_count && braked[index] == brake_count);
        for (uint8_t other = index + 1; other < 4; other++)
            assert(controllers_manager_is_running(other));
        unsigned writes = driven[index];
        tick();
        assert(driven[index] == writes && !locked);
    }
    puts("Single-motor and platform-mask stops passed with and without PID control");
    return 0;
}
