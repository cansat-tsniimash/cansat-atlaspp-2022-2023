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
#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api.h>
#include <nRF24L01_PL/nrf24_defs.h>
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
	CMD_OFF = 0x38,
	//gps read
	CMD_Read_gps = 0x39,
	//запись бита полета ёмаё | write fly's bit
	CMD_Write_flys_bit = 0x40,
	//отправка по радио эсть жэ || radio send
	CMD_Radio_send = 0x41,
	//отправка по радио записоного сообщения
	CMD_Radio_send_d = 0x42,
	//настройка радио
	CMD_Settings = 0x43
} cmd_t;
#pragma pack(push,1)
typedef struct
{
	nrf24_data_rate_t data_rate;
	nrf24_tx_power_t tx_power;
	uint8_t rf_channel;
	nrf24_crc_size_t crc_size;
	nrf24_address_width_t address_width;
	bool en_dyn_payload_size;
	bool en_ack_payload;
	bool en_dyn_ack;
	uint64_t tx_chanel;
	//мб удалить
	uint8_t auto_retransmit_count;
	uint8_t auto_retransmit_delay;
} settings_pack_t;
#pragma pack(pop)
typedef enum i2c_link_cmd_t
{
    I2C_LINK_CMD_NONE = 0x00,
    I2C_LINK_CMD_GET_SIZE = 0x01,
    I2C_LINK_CMD_GET_PACKET = 0x02,
    I2C_LINK_CMD_SET_PACKET = 0x04,
} its_i2c_link_cmd_t;
//buzzer on/off
//args: i2c address, onoff: true/false
//ret: hal error
int bb_buzzer_control(uint16_t I2C_ADDRES, bool onoff);
//chip erraze
//args: i2c address
//ret: hal error
int bb_chip_err(uint16_t I2C_ADDRES);
//gps erraze
//args: i2c address
//ret: hal error
int bb_gps_err(uint16_t I2C_ADDRES);
//request to read data from Black boxes
//args: i2c address, size of data 1 byte
// Continue? true/false
//ret: hal error
int bb_read_req(uint16_t I2C_ADDRES, uint8_t size, bool is_continue);
//read from black boxes
//args: i2c address
// buffer 1 byte
// size of data 1 byte
//ret: hal error
int bb_read(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size);
//write to black boxes
//args: i2c address, buffer 1 byte
// size of data 1 byte
//ret: hal error
int bb_write(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size);
//read request to black boxes with addres
//args: i2c address, addres 4 byte
//size of data 1 byte
//ret: hal error
int bb_read_req_addr(uint16_t I2C_ADDRES, uint32_t addr, uint8_t size);
//read from black boxes with addres
//args: i2c address, addres 4 byte
//buffer 1 byte
//size of data 1 byte
//ret: hal error
int bb_read_addr(uint16_t I2C_ADDRES, uint32_t *addr, uint8_t* buf, uint8_t size);
//turn off the power
//args: i2c address
//ret: hal error
int bb_off(uint16_t I2C_ADDRES);
//request to read gps
//args: i2c address, number of gps addres 2 bytes
//ret: hal error
int bb_read_gps_req(uint16_t I2C_ADDRES, uint16_t num);
//read gps
//args: i2c address, number of gps addres 2 bytes
// buffer 1 byte
// size of data 1 byte
//ret: hal error
int bb_read_gps(uint16_t I2C_ADDRES, uint16_t *num, uint8_t* buf, uint8_t size);
//write fly's bit
//args: i2c address, on/off true/false
//ret: hal error
int bb_write_flys_bit(uint16_t I2C_ADDRES, bool onoff);
//send data by radio
//args: i2c address, buffer 1 byte
// size 1 byte
//ret: hal error
int bb_radio_send(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size);
//send data by radio default
//args: i2c address, buffer 1 byte
// size 1 byte
//ret: hal error
int bb_radio_send_d(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size);
//radio settings pack
//args: i2c address, settings pack struct
//ret: hal error
int bb_settings_pack(uint16_t I2C_ADDRES, settings_pack_t *settings_pack);
int bb_ping(uint16_t I2C_ADDRES);
#endif /* BB_H_ */
