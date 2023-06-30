/*
 * Application.c
 *
 *  Created on: Jan 14, 2023
 *      Author: Install
 */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "mem-proxy.h"


#include "drivers_i2c/Inc/its-i2c-link.h"
#include <MX25L512_/MX25L512_up.h>

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;


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

#pragma pack(pop)


void off_bb(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}

void on_bb(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}

void buzzer_control(bool onoff){
	if (onoff)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

typedef enum
{
	SYS_TEST_MEM_DATA = (1 << 0),
	SYS_TEST_NRF24 = (1 << 1),
	SYS_TEST_MEM_GPS = (1 << 2)
} sys_check_t;

int all_systems_check(bus_t *bus_data, int comp)
{
	int ret = 0;
	if (comp & SYS_TEST_MEM_DATA)
	{
		uint8_t id[3] = {0};
		mx25l512_rdid(bus_data, id);
		if ((id[0] == 0xc2) && (id[1] == 0x20) && (id[2] == 0x10))
			ret |= SYS_TEST_MEM_DATA;
	}
	return ret;
}

uint8_t read_buf[256];

int app_main(){
	on_bb();

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
	its_i2c_link_start();
	cmd_pack_t pack;

	uint32_t addr_read = 0;
	uint32_t addr_write = 0;

	bus_t bus;
	mx25l512_spi_pins_t mx25_pins;
	mx25_pins.cs_port = GPIOB;
	mx25_pins.cs_pin = GPIO_PIN_1;
	mx25l512_spi_init(&bus, &hspi1, &mx25_pins);
	memproxy_init(&bus);

	volatile int check = all_systems_check(&bus,  SYS_TEST_MEM_DATA);
	buzzer_control(true);
	HAL_Delay(500);
	buzzer_control(false);
	HAL_Delay(300);
	for (int i = 0; i < check; i++)
	{
		buzzer_control(true);
		HAL_Delay(300);
		buzzer_control(false);
		HAL_Delay(300);
	}

	uint16_t a = 0;
	uint16_t period = 200;
	int check_i = 0;

	while(1){
		if (a > period)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			a = 0;

			if (check_i == check * 2)
			{
				period = 1500;
				check_i = 0;
			}
			else
			{
				period = 200;
				check_i++;
			}

		}
		a++;
		HAL_Delay(1);

		int rc = its_i2c_link_read(&pack, sizeof(pack));

	/*	for (int i = 0 ; i < 100; i++)
		{
			volatile uint8_t buffer[200];
			memset(buffer, i, 200);
			memproxy_write(buffer, 200);
			volatile int x = 0;
		}

		for (int i = 0 ; i < 100; i++)
		{
			volatile uint8_t buffer[0x100];
			memset(buffer, 0xcc, 0x100);
			uint32_t addr = 0x100*i;
			mx25l512_read(&bus, &addr, buffer, 0x100);

			volatile int x = 0;
		}*/

		/*while(1)
		{volatile int check = all_systems_check(&bus,  SYS_TEST_MEM_DATA);}*/

		if (rc > 0)
		{
			switch(pack.num){
				case CMD_BUZ:
					if (pack.size == 1)
					{
						buzzer_control(pack.data[0]);
					}
					break;
				case CMD_CE:
					if (pack.size == 0){
						addr_write = 0;
						mx25l512_CE_up(&bus, 10);//Затираю чип целиком
					}
					break;

				case CMD_ReadADDR:
					if ((pack.size = 5) && (pack.data[4] <= 32))
					{
						uint32_t addr = (pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24) & 0xFF00;
						uint8_t size = pack.data[4];
						mx25l512_read(&bus, &addr, read_buf, 256);//читаю данные
						memcpy(pack.data + 4, read_buf + ((pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24) & 0xFF), size);
						pack.size = size + 4;
						its_i2c_link_write(&pack, sizeof(pack));
					}
					break;
				case CMD_Write:
					if (pack.size <= 32)
					{
						memproxy_write(pack.data, pack.size);
					}
					break;
				case CMD_Read:
					if (pack.size == 1 && pack.data[0] <= 32)
					{
						addr_read = 0x0000;
						uint8_t size = pack.data[0];
						mx25l512_read(&bus, &addr_read, read_buf, 256);//читаю данные
						memcpy(pack.data, read_buf, size);
						addr_read = size;
						pack.size = size;
						its_i2c_link_write(&pack, sizeof(pack));

					}
					break;
				case CMD_Continue:
					if (pack.size == 1 && pack.data[0] <= 32)
					{
						uint8_t size = pack.data[0];
						if(addr_read + size < 0xffff){
							uint32_t new_addr = addr_read & 0xFF00;
							mx25l512_read(&bus, &new_addr, read_buf, 256);//читаю данные
							if (((addr_read & 0xFF) + size) > 0xFF)
							{
								uint8_t part = 0xFF - (addr_read & 0xFF) + 1;
								memcpy(pack.data, read_buf + (addr_read & 0xFF), part);
								new_addr = (addr_read + size) & 0xFF00;
								mx25l512_read(&bus, &new_addr, read_buf, 256);
								memcpy(pack.data + part, read_buf, size - part);
							}
							else
							{
								memcpy(pack.data, read_buf + (addr_read & 0xFF), size);
							}
							addr_read = addr_read + size;
							pack.size = size;
							its_i2c_link_write(&pack, sizeof(pack));
						}
					}
					break;
				case CMD_OFF:
					if (pack.size == 0)
					{
						off_bb();
					}
					break;
				}
			}
		}
}
