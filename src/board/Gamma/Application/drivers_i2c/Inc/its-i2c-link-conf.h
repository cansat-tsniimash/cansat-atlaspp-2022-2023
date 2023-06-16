
#ifndef ITS_I2C_LINK_CONF_H_
#define ITS_I2C_LINK_CONF_H_

#include <stm32f0xx_hal.h>

//! Адрес ведомого в 7мибитном формате, с выравниванием по правому краю
#define I2C_LINK_ADDR (0x78)

// Проритеты I2C прерываний
#define I2C_LINK_IRQn I2C1_IRQn

#define I2C_LINK_IRQ_PRIORITY 1

//! Размер пакета передаваемого/принимаемого за одну транзакцию
#define I2C_LINK_MAX_PACKET_SIZE (38)

//! Количество приёмных буферов (каждый по I2C_LINK_PACKET_SIZE байт)
#define I2C_LINK_RXBUF_COUNT (2)
//! Количество отправных буферов (каждый по I2C_LINK_PACKET_SIZE байт)
#define I2C_LINK_TXBUF_COUNT (2)

//! LL i2c-handle (I2C_Typedef)
#define I2C_LINK_BUS_HANDLE (I2C1)
#define I2C_LINK_DMA_HANDLE (DMA1)
#define I2C_LINK_DMA_CHANNEL_TX (LL_DMA_CHANNEL_2)
#define I2C_LINK_DMA_CHANNEL_RX (LL_DMA_CHANNEL_3)

#define I2C_LINK_PINS_AF LL_GPIO_AF_4

#define I2C_LINK_SDA_PORT GPIOA
#define I2C_LINK_SDA_PIN LL_GPIO_PIN_10

#define I2C_LINK_SCL_PORT GPIOA
#define I2C_LINK_SCL_PIN LL_GPIO_PIN_9




#endif /* ITS_I2C_LINK_CONF_H_ */
