//------------------------------------------------------------
// File name: hardware_i2c.c
//------------------------------------------------------------

#include "hardware_i2c.h"
#include "message_queue.h"
#include <cmsis_os2.h>

I2C_HandleTypeDef hi2c2;
message_queue_t I2CCommandQueue;
i2c_receive_state_t receive_state;

uint8_t i2c_send_buffer[256];

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

    // Awaint for the first byte of the message
    receive_state = AWAITING_SIZE;
    HAL_I2C_EnableListen_IT(&hi2c2);
}

void send_external_i2c(uint8_t* data, uint16_t data_len)
{
    // We need to wait here since master may not yet request data from slave
    // and we will not be able to send data until then (receive_state == LISTENING)
    // TODO: Replace better wait mechanism
    unsigned int timeout = 1000;
    i2c_receive_state_t state = receive_state;
    while(state != LISTENING)
    {
        osDelay(1);
        if(timeout-- == 0) {
            return;
        }
        state = receive_state;
    }

    if (state == LISTENING) 
    {
        unsigned int value = 128;

        uint16_t bytes_to_send = sizeof(value);
        memcpy(i2c_send_buffer, &value, bytes_to_send);

        if(HAL_I2C_Slave_Seq_Transmit_IT(&hi2c2, i2c_send_buffer, bytes_to_send, I2C_LAST_FRAME) != HAL_OK){
            // TODO: Handle error here
        }
    }
}

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

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance == I2C2)
  {
    receive_state = AWAITING_SIZE;
    HAL_I2C_EnableListen_IT(&hi2c2);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // TODO: Handle error here
    // For now, just re-enable the I2C listen mode
    HAL_I2C_EnableListen_IT(hi2c);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}