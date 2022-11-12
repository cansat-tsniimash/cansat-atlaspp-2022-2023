/*
 * i2c-crutch.c
 *
 *  Created on: 12 нояб. 2022 г.
 *      Author: Agnus
 */

#include <stdint.h>
#include "stm32f0xx_hal.h"


static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
 {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;

    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) // Wait until flag is set
    {
        if(timeout != HAL_MAX_DELAY) // Check for the timeout
        {
            if((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) ret = 0;
        }

        asm("nop");
    }
    return ret;
}

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout)
{
        // 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 1. Clear PE bit.
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

    //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_I2C_DeInit(hi2c);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    GPIO_InitStructure.Pin = GPIO_PIN_8; // SCL // если пин другой, то укажите нужный
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); // если порт другой, то укажите нужную букву GPIOх, и ниже там все порты и пины поменяйте на своё

    GPIO_InitStructure.Pin = GPIO_PIN_9; // SDA
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. Check SCL and SDA High level in GPIOx_IDR.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_8, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_9, GPIO_PIN_SET, timeout);

    // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    // 5. Check SDA Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET, timeout);

    // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

    // 7. Check SCL Low level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET, timeout);

    // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

    // 9. Check SCL High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_8, GPIO_PIN_SET, timeout);

    // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    // 11. Check SDA High level in GPIOx_IDR.
    wait_for_gpio_state_timeout(GPIOB, GPIO_PIN_9, GPIO_PIN_SET, timeout);

    // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    //GPIO_InitStructure.Alternate = GPIO_AF4_I2C2; // F4

    GPIO_InitStructure.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 13. Set SWRST bit in I2Cx_CR1 register.
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 14. Clear SWRST bit in I2Cx_CR1 register. */
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    asm("nop");

    /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    asm("nop");

    // Call initialization function.
    HAL_I2C_Init(hi2c);
}

void reset_i2c_1()
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_Delay(100);
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();
	HAL_Delay(100);
}