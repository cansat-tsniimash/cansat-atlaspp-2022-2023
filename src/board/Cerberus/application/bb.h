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

int bb_buzzer_control(bool onoff);
int bb_chip_err();
int bb_read_req(uint8_t size, bool is_continue);
int bb_read_addr(uint32_t *addr, uint8_t* buf, uint8_t size);
int bb_write(uint8_t* buf, uint8_t size);
int bb_read_req_addr(uint32_t addr, uint8_t size);
int bb_read_addr(uint32_t *addr, uint8_t* buf, uint8_t size);
int bb_off();
int bb_read_gps_req(uint16_t num);
int bb_read_gps(uint16_t *num, uint8_t* buf, uint8_t size);
int bb_write_flys_bit(bool onoff);
int bb_write_flys_bit_d(bool onoff);
int bb_radio_send(uint8_t* buf, uint8_t size);
int bb_radio_send_d(uint8_t* buf, uint8_t size);
int bb_settings_pack(settings_pack_t *settings_pack);
#endif /* BB_H_ */
