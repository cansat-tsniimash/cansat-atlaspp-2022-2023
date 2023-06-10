/*
 * Application.c
 *
 *  Created on: Jan 14, 2023
 *      Author: Install
 */

#include <stdio.h>
#include <string.h>

#include "main.h"


#include "drivers_i2c/Inc/its-i2c-link.h"
#include <MX25L512_/MX25L512.h>

extern I2C_HandleTypeDef hi2c1;


uint8_t flag = 16;
#pragma pack(push,1)

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
	    //отправка по радио записаного сообщения
	    CMD_Radio_send_d = 0x42,
	    //настройка радио
	    CMD_Settings = 0x43
	}cmd_t;

uint8_t buf[30];

typedef struct
{
	uint8_t num;
	uint8_t size;
	uint8_t data[36];
} cmd_pack_t;

typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint16_t time_s;
	float lat;
	float lon;
	float alt;
	uint32_t gps_time_s;
	uint32_t gps_time_us;
	int8_t fix;
	uint16_t crc;
} nrf_pack_t;

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
	uint64_t tx_channel;
	//мб удалить
	uint8_t auto_retransmit_cout;
	uint8_t auto_retransmit_delay;
}settings_pack_t;

#pragma pack(pop)




int app_main(){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	its_i2c_link_start();

//variables

//settings





	int nrf_irq;
	uint32_t start_time_nrf = HAL_GetTick();
	cmd_pack_t pack;
	nrf_pack_t nrf_pack = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

	uint8_t Data[3];
	uint32_t addr;
	int fix_;
	int64_t cookie;
	uint64_t time_s_;
	uint32_t time_us_;
	float lat;
	float lon;
	float alt;



	settings_pack_t settings_pack;
	while(1){
		//mx25l512_rdid(&bus_data, Data);
		//HAL_Delay(10);

		HAL_Delay(1);
		//shift_reg_write_bit_8(&shift_reg, 7, 1);
		//HAL_Delay(100);
		//shift_reg_write_bit_8(&shift_reg, 7, 0);
		//HAL_Delay(100);
		//const char hello[] = "hello i'm a bus";
		//int rrc = its_i2c_link_write(hello, sizeof(hello));

		int rc = its_i2c_link_read(&pack, sizeof(pack));
		if (rc > 0)
				{
					switch(pack.num){
						case CMD_BUZ:
							if (pack.size == 1)
							{
								if (pack.data[0])
									shift_reg_write_bit_8(&shift_reg, 7, 1);//Вкл
								else
									shift_reg_write_bit_8(&shift_reg, 7, 0);//Выкл
							}
							break;
						case CMD_CE:
							if (pack.size == 0)
								mx25l512_CE_up(&bus_data, 10);//Затираю чип целиком
							break;

						case CMD_ReadADDR:
							if ((pack.size = 5) && (pack.data[4] <= 32))
							{
								uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
								uint8_t size = pack.data[4];
								mx25l512_read(&bus_data, &addr, pack.data + 4, size);//читаю данные
								pack.size = size + 4;
								its_i2c_link_write(&pack, sizeof(pack));
							}
							break;
						case CMD_Write:
							if (pack.size <= 32)
							{
								addr_write = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
								mx25l512_PP_up(&bus_data, &addr_write, pack.data + 4, pack.size - 4, 10);//Записываю данные
							}
							break;
						case CMD_Read:
							if (pack.size == 1 && pack.data[0] <= 32)
							{
								addr_read = 0x0000;
								uint8_t size = pack.data[0];
								mx25l512_read(&bus_data, &addr_read, pack.data, size);
								addr_read = size << 4;
								pack.size = size;
								its_i2c_link_write(&pack, sizeof(pack));

							}
							break;
						case CMD_Continue:
							if (pack.size == 1 && pack.data[0] <= 32)
							{
								uint8_t size = pack.data[0];
								uint32_t new_addr = addr + (size << 4);
								if ((addr_read & (0x0f << 12)) != (new_addr & (0x0f << 12)))
								{
									addr_read = new_addr & (0x0f << 12);
									new_addr = addr_read + (sizeof(pack) << 4);
								}
								mx25l512_read(&bus_data, &addr_read, pack.data, size);
								addr_read = new_addr;
								pack.size = size;
								its_i2c_link_write(&pack, sizeof(pack));
							}
							break;
						case CMD_OFF:
							if (pack.size == 0)
							{
								shift_reg_write_bit_8(&shift_reg, 6, 0);
							}
							break;

						case CMD_Write_flys_bit:
							if (pack.size == 1)
							{

							}
							break;




						}
					}







}
