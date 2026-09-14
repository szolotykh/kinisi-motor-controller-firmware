//------------------------------------------------------------
// File name: hardware_i2c.c
// Description: Receive framed I2C requests and transmit replies without blocking the command task.
//------------------------------------------------------------

#include "hardware_i2c.h"
#include "message_queue.h"
#include <cmsis_os2.h>

I2C_HandleTypeDef hi2c2;
message_queue_t I2CCommandQueue;
volatile i2c_receive_state_t receive_state;
static volatile uint8_t tx_busy;

uint8_t i2c_send_buffer[256];

/**
 * @brief Configure I2C2 as a listening slave and reset receive/transmit state.
 */
void initialize_external_i2c(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct = {0};

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    // I2C2 GPIO Configuration  
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;  // PB10 -> I2C2_SCL, PB11 -> I2C2_SDA
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize I2C2
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;       // 100KHz clock speed
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 8 << 1;     // Device address shifted left by 1 bit. Here it's 8.
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&hi2c2) != HAL_OK){
        return;
    }

    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);  // Set priority
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);          // Enable the I2C2 event interrupt
    // A master's final read NACK completes a slave transmit through the error IRQ.
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

    // Await the first byte of the message.
    tx_busy = 0;
    receive_state = AWAITING_SIZE;
    HAL_I2C_EnableListen_IT(&hi2c2);
}

/**
 * @brief Copy a frame and start an interrupt-driven transfer when a master is reading.
 * @param data Complete length-prefixed frame to copy.
 * @param data_len Frame size in bytes.
 * @return Nonzero if accepted; zero when busy, unavailable, oversized, or HAL rejects it.
 * @note Does not wait for a master transaction; preserves the caller interrupt mask.
 */
uint8_t try_send_external_i2c(uint8_t* data, uint16_t data_len)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t sent = 0;
    if (receive_state == LISTENING && !tx_busy && data_len <= sizeof(i2c_send_buffer)) {
        memcpy(i2c_send_buffer, data, data_len);
        // Wire masters choose a fixed read size before seeing the length prefix.
        // Supply zero padding after a shorter ACK/ERROR instead of stretching SCL
        // indefinitely when HAL exhausts the frame. The master's NACK releases
        // the unused tail through the existing error/listen completion callbacks.
        memset(i2c_send_buffer + data_len, 0, sizeof(i2c_send_buffer) - data_len);
        tx_busy = 1;
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, i2c_send_buffer, sizeof(i2c_send_buffer), I2C_LAST_FRAME) == HAL_OK)
            sent = 1;
        else tx_busy = 0;
    }
    __set_PRIMASK(mask);
    return sent;
}

/**
 * @brief Select transmit readiness or begin receiving the next length-prefixed request.
 * @note Runs from the I2C interrupt handler.
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
  // Master requests data from slave
    if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        receive_state = LISTENING;
    } else {
        // Master sends data to slave
        //HAL_I2C_Slave_Receive_IT(hi2c, &RxData, 1);
        receive_state = AWAITING_SIZE;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_message_buffer, 1, I2C_FIRST_FRAME);
    }
}

/**
 * @brief Advance length/payload reception and enqueue a completed I2C frame.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2)
    {
        if(receive_state == AWAITING_SIZE) {
            // Received first byte of message, which is the size byte
            // Set the receive state to AWAITING_MESSAGE and wait for the rest of the message
            // i2c_message_buffer[0] is now the size of the message
            receive_state = AWAITING_MESSAGE;
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_message_buffer + 1, i2c_message_buffer[0], I2C_LAST_FRAME);
        } else {
            enqueue(&I2CCommandQueue, i2c_message_buffer);
        }
    }
}

/**
 * @brief Release the transmit buffer and re-arm slave listening after a transaction.
 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance == I2C2)
  {
    tx_busy = 0;
    receive_state = AWAITING_SIZE;
    HAL_I2C_EnableListen_IT(&hi2c2);
  }
}

/**
 * @brief Reset transfer state and re-arm slave listening after a HAL transport error.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) return;
    tx_busy = 0;
    receive_state = AWAITING_SIZE;
    // Release an interrupted reply before accepting the next master transaction.
    HAL_I2C_EnableListen_IT(hi2c);
}

/**
 * @brief Forward I2C2 event interrupts to the HAL state machine.
 */
void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

/**
 * @brief Forward read-completion NACKs and bus errors to the HAL state machine.
 * @note The HAL listen/error callbacks release tx_busy and re-arm the slave.
 */
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c2);
}
