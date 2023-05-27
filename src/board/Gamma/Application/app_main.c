/*
 * Application.c
 *
 *  Created on: Jan 14, 2023
 *      Author: Install
 */

#include <stdio.h>

#include "main.h"

#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api.h>
#include <nRF24L01_PL/nrf24_defs.h>
#include "drivers_i2c/Inc/its-i2c-link.h"
#include <MX25L512_/MX25L512.h>
extern SPI_HandleTypeDef hspi1;
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
#pragma pack(pop)

void off_bb(shift_reg_t *shift_reg){
	shift_reg_write_bit_8(shift_reg, 3, true);
}
void buzzer_control(shift_reg_t *shift_reg, bool onoff){
	shift_reg_write_bit_8(shift_reg, 7, onoff);
}

int app_main(){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	its_i2c_link_start();

//variables
	uint64_t tx_adrr = 0xafafafaf01;
	uint8_t arr[32] = {1, 2, 3, 4, 5};
	uint8_t packet[32] = {1, 2, 3};
	nrf24_fifo_status_t tx_status;
	nrf24_fifo_status_t rx_status;
//settings
	shift_reg_t shift_reg;
	shift_reg.bus = &hspi1;
	shift_reg.latch_port = GPIOB;
	shift_reg.latch_pin = GPIO_PIN_1;
	shift_reg.oe_port = GPIOA;
	shift_reg.oe_pin = GPIO_PIN_1;
	shift_reg_init(&shift_reg);
	shift_reg_write_8(&shift_reg, 0x70);

	nrf24_spi_pins_sr_t nrf_pins;
	nrf_pins.this = &shift_reg;
	nrf_pins.pos_CE = 5;
	nrf_pins.pos_CS = 4;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init_sr(&nrf24, &hspi1, &nrf_pins);

	nrf24_mode_power_down(&nrf24);
	nrf24_rf_config_t nrf_config;
	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_18_DBM;
	nrf_config.rf_channel = 77;
	nrf24_setup_rf(&nrf24, &nrf_config);
	nrf24_protocol_config_t nrf_protocol_config;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.en_dyn_payload_size = true;
	nrf_protocol_config.en_ack_payload = true;
	nrf_protocol_config.en_dyn_ack = true;
	nrf_protocol_config.auto_retransmit_count = 0;
	nrf_protocol_config.auto_retransmit_delay = 0;
	nrf24_setup_protocol(&nrf24, &nrf_protocol_config);
	nrf24_pipe_set_tx_addr(&nrf24, 0xafafafafaf);

	nrf24_pipe_config_t pipe_config;
	for (int i = 1; i < 6; i++)
	{
		pipe_config.address = 0xcfcfcfcfcf;
		pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
		pipe_config.enable_auto_ack = false;
		pipe_config.payload_size = -1;
		nrf24_pipe_rx_start(&nrf24, i, &pipe_config);
	}

	pipe_config.address = 0xafafafaf01;
	pipe_config.enable_auto_ack = false;
	pipe_config.payload_size = -1;
	nrf24_pipe_rx_start(&nrf24, 0, &pipe_config);

	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);

	bus_t bus_data;
	mx25l512_spi_pins_sr_t mx25_data_pins;
	mx25_data_pins.this = &shift_reg;
	mx25_data_pins.pos_CS = 6;
	mx25l512_spi_init_sr(&bus_data, &hspi1, &mx25_data_pins);

	int nrf_irq;
	uint32_t start_time_nrf = HAL_GetTick();
	cmd_pack_t pack;
	nrf_pack_t nrf_pack = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

	while(1){

		int rc = its_i2c_link_read(&pack, sizeof(pack));
		if (rc > 0)
		{
			/*switch(pack.num){
				case CMD_BUZ:
					if (pack.size == 1)
					{
						buzzer_control(&shift_reg, pack.data[0])
					}
					break;
				case CMD_CE:
					if (pack.size == 0)
						mx25l512_CE(&bus_data);//Затираю чип целиком
					break;
				case CMD_CE_GPS:
					if (pack.size == 0)
						mx25l512_CE(&bus_gps);//Затираю чип целиком
					break;
				case CMD_ReadADDR:
					if ((pack.size = 5) && (pack.data[4] <= 32))
					{
						uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						uint8_t size = pack.data[4];
						mx25l512_read(&bus_data, &addr, pack.data, size);//читаю данные
						pack.size = size;
						its_i2c_link_write(&pack, sizeof(pack));
					}
					break;
				case CMD_Write:
					if (pack.size <= 32)
					{
						//uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						//mx25l512_PP(&bus_data, &addr, pack.data + 4, pack.size - 4);//Записываю данные
					}
					break;
				case CMD_Read:
					if (pack.size == 1 && pack.data[0] <= 32)
					{
						addr = 0x0000;
						uint8_t size = pack.data[0];
						mx25l512_read(&bus_data, &addr, pack.data, size);
						addr = size << 4;
						its_i2c_link_write(&pack, sizeof(pack));

					}
				case CMD_Continue:
					if (pack.size == 1 && pack.data[0] <= 32)
					{
						uint8_t size = pack.data[0];
						uint8_t new_addr = addr + (size << 4);
						if (addr && (0x0f << 12) != new_addr && (0x0f << 12))
						{
							addr = new_addr && (0x0f << 12);
							new_addr = addr + (sizeof(pack) << 4);
						}
						mx25l512_read(&bus_data, &addr, pack.data, size);
						addr = new_addr;
						its_i2c_link_write(&pack, sizeof(pack));
					}
				case CMD_OFF:
					if (pack.size = 0)
					{
						shift_reg_write_bit_8(&shift_reg, 6, 0);
					}
				case CMD_Read_gps:
					if (pack.size = 2)
					{

						uint8_t num = pack.num;
						uint32_t addr = (num % 9* 28) << 4 | (num / 9) << 12;

						mx25l512_read(&bus, addr, pack.data, 28);
					}
				case CMD_Write_flys_bit:
					if (pack.size = 1)
					{

					}

				case CMD_Radio_send:
					if (pack.size <= 32)
					{
						nrf24_fifo_write(&nrf24, &pack.data, sizeof(&pack.data), false);
					}
				case  CMD_Radio_send_d:
					if(pack.size <= 32)
					{
						memcpy(buf, i2c_pack.data, i2c_pack.size);
					}
				case CMD_Settings:
					if(pack.size = 18)
					{

					}

				}*/
			}





		int comp;
		nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
		if (tx_status != NRF24_FIFO_FULL){
				nrf24_fifo_write(&nrf24, (uint8_t *)&nrf_pack, sizeof(nrf_pack), false);
				start_time_nrf = HAL_GetTick();
		}
		else
		if (HAL_GetTick()-start_time_nrf >= 100)
		{
			nrf24_fifo_flush_tx(&nrf24);
			nrf24_fifo_write(&nrf24, (uint8_t *)&nrf_pack, sizeof(nrf_pack), false);
			start_time_nrf = HAL_GetTick();
		}
		nrf24_irq_get(&nrf24, &comp);
		nrf24_irq_clear(&nrf24, comp);
	}

}