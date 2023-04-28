/*
 * bb.h
 *
 *  Created on: Mar 29, 2023
 *      Author: Install
 */

#ifndef BB_H_
#define BB_H_

#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "i2c-crutch.h"
#include "string.h"

typedef enum
{
	CMD_BUZ = 0x31,
	//управление пьезодинамиком
	CMD_CE = 0x32,
	//очистка памяти
	CMD_Read = 0x33,
	//чтение ПАМЯТИ
	CMD_Write = 0x34,
	//запись данных
	CMD_ReadADDR = 0x35,
	//Чтение по АДРЕСУ
	CMD_Continue = 0x36,
	//Продолжаю
	CMD_CE_GPS = 0x37,
	//выключить питание
	CMD_OFF = 0x38

} cmd_t;


typedef enum i2c_link_cmd_t
{
    I2C_LINK_CMD_NONE = 0x00,
    I2C_LINK_CMD_GET_SIZE = 0x01,
    I2C_LINK_CMD_GET_PACKET = 0x02,
    I2C_LINK_CMD_SET_PACKET = 0x04,
} its_i2c_link_cmd_t;

int bb_buzzer_control(bool onoff);
int bb_chip_err();
int bb_write(uint32_t addr, uint8_t* buf, uint8_t size);
int bb_read_req(uint32_t addr, uint8_t size);
int bb_read(uint32_t *addr, uint8_t* buf, uint8_t size);

#endif /* BB_H_ */
