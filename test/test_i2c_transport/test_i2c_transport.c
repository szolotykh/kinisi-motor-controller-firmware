//------------------------------------------------------------
// File name: test_i2c_transport.c
// Description: Exercise production I2C IRQ wiring and buffer ownership across master reads.
//------------------------------------------------------------
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Include the implementation once: its existing public header defines the receive buffer.
#include "../../lib/hardware/hardware_i2c.c"

static uint32_t irq_mask;
static bool enabled[TEST_IRQ_COUNT], priority_set[TEST_IRQ_COUNT];
static unsigned listen_count, event_count, error_count;
static bool transfer_active, bytes_complete, bus_error;
static uint8_t *active_buffer;
static uint16_t active_length;
static HAL_StatusTypeDef transmit_result = HAL_OK;

/** @brief Return the fixture interrupt mask. */
uint32_t __get_PRIMASK(void) { return irq_mask; }
/** @brief Mask interrupts while production code owns the shared transmit buffer. */
void __disable_irq(void) { irq_mask = 1; }
/** @brief Restore the caller's interrupt state. */
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }

/** @brief Require both IRQ lines to be configured at the same priority. */
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq < TEST_IRQ_COUNT && priority == 0 && subpriority == 0);
    priority_set[irq] = true;
}

/** @brief Record NVIC enablement so simulated interrupts use the actual configuration. */
void HAL_NVIC_EnableIRQ(IRQn_Type irq)
{
    assert(irq < TEST_IRQ_COUNT && priority_set[irq]);
    enabled[irq] = true;
}

/** @brief Check that initialization still selects the external I2C pins. */
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config)
{
    assert(port == GPIOB && config->Pin == (GPIO_PIN_10 | GPIO_PIN_11));
}

/** @brief Accept initialization for the external I2C instance. */
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *handle)
{
    assert(handle->Instance == I2C2);
    return HAL_OK;
}

/** @brief Track HAL listen rearming, including after the final read NACK. */
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *handle)
{
    assert(handle == &hi2c2);
    ++listen_count;
    return HAL_OK;
}

/** @brief Retain the production buffer until completion, matching HAL asynchronous ownership. */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *handle,
    uint8_t *data, uint16_t length, uint32_t options)
{
    assert(handle == &hi2c2 && irq_mask == 1 && options == I2C_LAST_FRAME);
    assert(!transfer_active);
    if (transmit_result != HAL_OK) return transmit_result;
    transfer_active = true;
    bytes_complete = false;
    active_buffer = data;
    active_length = length;
    return HAL_OK;
}

/** @brief Accept the setup for a master's write transaction. */
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *handle,
    uint8_t *data, uint16_t length, uint32_t options)
{
    assert(handle == &hi2c2 && data && length > 0);
    assert(options == I2C_FIRST_FRAME || options == I2C_LAST_FRAME);
    return HAL_OK;
}

/** @brief Finish sending bytes; HAL still awaits the master's NACK to finish listening. */
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *handle)
{
    assert(handle == &hi2c2 && transfer_active);
    ++event_count;
    bytes_complete = true;
}

/**
 * @brief Model the HAL callbacks reached through the error IRQ.
 * @note STM32F4 HAL's I2C_Slave_AF calls ListenCpltCallback for an I2C_LAST_FRAME
 * read with XferCount zero; a premature NACK or bus error reaches ErrorCallback.
 * This is a focused callback model, not an emulation of the peripheral registers.
 */
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *handle)
{
    assert(handle == &hi2c2 && transfer_active);
    ++error_count;
    transfer_active = false;
    if (bus_error || !bytes_complete) HAL_I2C_ErrorCallback(handle);
    else HAL_I2C_ListenCpltCallback(handle);
}

/** @brief Deliver final NACK only through an enabled NVIC error interrupt. */
static void master_finish_read(void)
{
    if (enabled[I2C2_EV_IRQn]) I2C2_EV_IRQHandler();
    if (enabled[I2C2_ER_IRQn]) I2C2_ER_IRQHandler();
}

/** @brief Address the slave for reading, then require the next frame to be accepted. */
static void start_read(uint8_t *frame, uint16_t length)
{
    HAL_I2C_AddrCallback(&hi2c2, I2C_DIRECTION_RECEIVE, 16);
    assert(try_send_external_i2c(frame, length));
    assert(active_length == length && memcmp(active_buffer, frame, length) == 0);
}

/** @brief Verify consecutive INIT, sync, READY, and ordinary replies release the buffer. */
static void test_consecutive_reads(void)
{
    uint8_t replies[][5] = {{4, 0x70, 1, 0, 1}, {3, 0x71, 2, 0, 0},
                            {4, 0x73, 1, 0, 1}, {3, 0x20, 3, 0, 0}};
    for (unsigned i = 0; i < sizeof(replies) / sizeof(replies[0]); ++i) {
        start_read(replies[i], replies[i][0] + 1);
        uint8_t replacement[] = {3, 0x10, 99, 0};
        assert(!try_send_external_i2c(replacement, sizeof(replacement)));
        assert(memcmp(active_buffer, replies[i], active_length) == 0);
        unsigned previous_listens = listen_count;
        master_finish_read();
        assert(listen_count == previous_listens + 1);
        assert(!try_send_external_i2c(replacement, sizeof(replacement)));
    }
    assert(event_count == 4 && error_count == 4);
}

/** @brief Verify errors and rejected HAL calls release ownership for another transaction. */
static void test_transfer_recovery(void)
{
    uint8_t reply[] = {3, 0x20, 4, 0};
    start_read(reply, sizeof(reply));
    unsigned previous_listens = listen_count;
    I2C_HandleTypeDef unrelated_bus = {.Instance = (void *)(uintptr_t)1};
    HAL_I2C_ErrorCallback(&unrelated_bus);
    assert(listen_count == previous_listens);
    assert(!try_send_external_i2c(reply, sizeof(reply)));
    // A master ending a read early reaches the HAL error callback, not listen completion.
    I2C2_ER_IRQHandler();
    assert(listen_count == previous_listens + 1);
    start_read(reply, sizeof(reply));
    bus_error = true;
    I2C2_ER_IRQHandler();
    bus_error = false;
    start_read(reply, sizeof(reply));
    master_finish_read();

    HAL_I2C_AddrCallback(&hi2c2, I2C_DIRECTION_RECEIVE, 16);
    transmit_result = HAL_BUSY;
    assert(!try_send_external_i2c(reply, sizeof(reply)));
    transmit_result = HAL_OK;
    assert(try_send_external_i2c(reply, sizeof(reply)));
    master_finish_read();

    irq_mask = 1;
    start_read(reply, sizeof(reply));
    assert(irq_mask == 1);
    irq_mask = 0;
    master_finish_read();
}

/** @brief Run regressions against the real transport callbacks and interrupt entry points. */
int main(void)
{
    initialize_external_i2c();
    assert(enabled[I2C2_EV_IRQn] && enabled[I2C2_ER_IRQn]);
    assert(listen_count == 1);
    test_consecutive_reads();
    test_transfer_recovery();
    assert(irq_mask == 0);
    puts("I2C IRQ wiring, consecutive replies, buffer ownership and error recovery passed");
    return 0;
}
