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
    CMD_0 = 0x30,
    //ну типа да. fixme
    CMD_1 = 0x31,
    //управление пьезодинамиком
    CMD_2 = 0x32,
    //очистка памяти
    CMD_3 = 0x33,
    //чтение памяти
    CMD_4 = 0x34
    //запись данных

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
int bb_read(uint32_t addr, uint8_t* buf, uint8_t size);

#endif /* BB_H_ */
