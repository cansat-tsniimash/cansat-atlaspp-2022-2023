/*
 * Application.c
 *
 *  Created on: Jan 14, 2023
 *      Author: Install
 */

#include <stdio.h>
#include <string.h>

#include "main.h"

#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api.h>
#include <nRF24L01_PL/nrf24_defs.h>
#include "drivers_i2c/Inc/its-i2c-link.h"
#include <MX25L512_/MX25L512_up.h>
#include <ATGM336H/nmea_gps.h>
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

uint16_t Crc16(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	while (len--) {
		crc ^= *buf++ << 8;
		for (uint8_t i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}


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

uint8_t buf[32];

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


void off_bb(shift_reg_t *shift_reg){
	shift_reg_write_bit_8(shift_reg, 6, true);
}

void on_bb(shift_reg_t *shift_reg){
	shift_reg_write_bit_8(shift_reg, 6, false);
}

void buzzer_control(shift_reg_t *shift_reg, bool onoff){
	shift_reg_write_bit_8(shift_reg, 3, onoff);
}


int app_main(){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	its_i2c_link_start();

//variables
	uint64_t tx_adrr = 0x123456789a;
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
	shift_reg_write_8(&shift_reg, 0x2E);
	on_bb(&shift_reg);
	buzzer_control(&shift_reg, false);
	shift_reg_oe(&shift_reg, false);





	gps_init();

	int64_t cookie;
	float lat;
	float lon;
	float alt;
	int fix;

	bus_t bus_data;
	mx25l512_spi_pins_sr_t mx25_data_pins;
	mx25_data_pins.this = &shift_reg;
	mx25_data_pins.pos_CS = 1;
	mx25l512_spi_init_sr(&bus_data, &hspi1, &mx25_data_pins);

	bus_t bus_gps;
	mx25l512_spi_pins_sr_t mx25_gps_pins;
	mx25_gps_pins.this = &shift_reg;
	mx25_gps_pins.pos_CS = 2;
	mx25l512_spi_init_sr(&bus_gps, &hspi1, &mx25_gps_pins);
	uint8_t byte_r = 0;
	uint8_t byte_w = 0x4A;
	uint32_t addr = 8;
	uint8_t stat_reg = 0;
	uint8_t data3[3] = {0 };

	while(1)
	{
		break;
		buzzer_control(&shift_reg, true);
		HAL_Delay(100);
		buzzer_control(&shift_reg, false);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		//uint8_t byte = 0;
		//volatile HAL_StatusTypeDef re = HAL_UART_Receive(&huart2, &byte, 1, 1000);
		//gps_push_byte(byte);
		//gps_work();
		//gps_get_coords(&cookie, &lat, &lon, &alt, &fix);

		mx25l512_CE_up(&bus_data, 1000);
		HAL_Delay(42);
		mx25l512_RES(&bus_data, &byte_r);
		mx25l512_rdsr(&bus_data, &stat_reg);
		mx25l512_read(&bus_data, &addr, &byte_r, 1);//читаю данные
		mx25l512_PP_up(&bus_data, &addr, &byte_w, 1, 1000);
		HAL_Delay(42);
		mx25l512_read(&bus_data, &addr, &byte_r, 1);//читаю данные
		byte_r = 0;

		mx25l512_CE_up(&bus_gps, 1000);
		HAL_Delay(42);
		mx25l512_RES(&bus_gps, &byte_r);
		mx25l512_rdsr(&bus_gps, &stat_reg);
		mx25l512_read(&bus_gps, &addr, &byte_r, 1);//читаю данные
		mx25l512_PP_up(&bus_gps, &addr, &byte_w, 1, 1000);
		HAL_Delay(42);
		mx25l512_read(&bus_gps, &addr, &byte_r, 1);//читаю данные
		byte_r = 0;

	}

	nrf24_spi_pins_sr_t nrf_pins;
	nrf_pins.this = &shift_reg;
	nrf_pins.pos_CE = 4;
	nrf_pins.pos_CS = 5;
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
	nrf24_pipe_set_tx_addr(&nrf24, 0xE7E7E7E7E7);

	nrf24_pipe_config_t pipe_config;
	for (int i = 1; i < 6; i++)
	{
		pipe_config.address = 0x123456789a;
		pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
		pipe_config.enable_auto_ack = false;
		pipe_config.payload_size = -1;
		nrf24_pipe_rx_start(&nrf24, i, &pipe_config);
	}

	pipe_config.address = 0x123456789a;
	pipe_config.enable_auto_ack = false;
	pipe_config.payload_size = -1;
	nrf24_pipe_rx_start(&nrf24, 0, &pipe_config);

	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);



	int nrf_irq;
	uint32_t start_time_nrf = HAL_GetTick();
	cmd_pack_t pack;

	uint8_t Data[3];
	int fix_;
	//int64_t cookie;
	uint64_t time_s_;
	uint32_t time_us_;
	//float lat;
	//float lon;
	//float alt;
	//cmd_pack_t pack;

	gps_init();
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);

	settings_pack_t settings_pack;


	nrf_pack_t nrf_pack;
    nrf_pack.flag = 0x0f;
    nrf_pack.num = 0;

    uint32_t addr_read = 0;
    uint32_t addr_write = 0;

	while(1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);

		gps_work();
		gps_get_coords(&cookie, &lat, &lon, &alt, &fix_);
		gps_get_time(&cookie, &time_s_, &time_us_);
 		nrf_pack.lat = lat;
		nrf_pack.lon = lon;
		nrf_pack.alt = alt;
		nrf_pack.fix = fix_;
		nrf_pack.gps_time_s = time_s_;
		nrf_pack.gps_time_us = time_us_;

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
				case CMD_CE_GPS:
					if (pack.size == 0)
						mx25l512_CE_up(&bus_gps, 10);//Затираю чип целиком
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
						for (int i = 0; i < size; i++)
							pack.data[i] = i;
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
						for (int i = 0; i < size; i++)
							pack.data[i] = i + 40;
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
				case CMD_Read_gps:
					if (pack.size == 2)
					{
						uint16_t num = *((uint16_t *)pack.data);
						uint32_t addr = (num % 9* 28) << 4 | (num / 9) << 12;
						mx25l512_read(&bus_gps, &addr, pack.data + 2, 28);
						pack.size = 28 + 2;
						for (int i = 0; i < 28; i++)
							pack.data[i] = 0xab;
						its_i2c_link_write(&pack, sizeof(pack));
					}
					break;
				case CMD_Write_flys_bit:
					if (pack.size == 1)
					{

					}
					break;

				case CMD_Radio_send:
					if (pack.size <= 32)
					{
						nrf24_fifo_write(&nrf24, pack.data, pack.size, false);
					}
					break;
				case  CMD_Radio_send_d:
					if(pack.size > 0 && pack.size <= 32)
					{
						memcpy(buf, pack.data, pack.size);

					}

					if(pack.size == 0)
					{
						nrf24_fifo_write(&nrf24, buf, sizeof(buf), false);
					}
					break;
				case CMD_Settings:
					if(pack.size == 18)
					{
						memcpy(&settings_pack, pack.data, pack.size);
						nrf_config.data_rate = settings_pack.data_rate;
						nrf_config.tx_power = settings_pack.tx_power;
						nrf_config.rf_channel = settings_pack.rf_channel;
						nrf24_setup_rf(&nrf24, &nrf_config);
						nrf_protocol_config.address_width = settings_pack.address_width;
						nrf_protocol_config.crc_size = settings_pack.crc_size;
						nrf_protocol_config.en_dyn_payload_size = settings_pack.en_dyn_payload_size;
						nrf_protocol_config.en_ack_payload = settings_pack.en_ack_payload;
						nrf_protocol_config.en_dyn_ack = settings_pack.en_dyn_ack;

						nrf_protocol_config.auto_retransmit_count = settings_pack.auto_retransmit_cout;
						nrf_protocol_config.auto_retransmit_delay = settings_pack.auto_retransmit_delay;
						nrf24_pipe_set_tx_addr(&nrf24, settings_pack.tx_channel);
						nrf24_setup_protocol(&nrf24, &nrf_protocol_config);

					}
					break;

				}
			}


		nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
		if (tx_status != NRF24_FIFO_FULL){
				nrf_pack.time_s = HAL_GetTick();
				nrf_pack.crc = Crc16((uint8_t *)&nrf_pack, sizeof(nrf_pack));
				nrf_pack.num++;
				nrf24_fifo_write(&nrf24, (uint8_t *)&nrf_pack, sizeof(nrf_pack), false);
				start_time_nrf = HAL_GetTick();
		}
		else
			if (HAL_GetTick()-start_time_nrf >= 100)
			{
				nrf24_fifo_flush_tx(&nrf24);
				nrf_pack.time_s = HAL_GetTick();
				nrf_pack.crc = Crc16((uint8_t *)&nrf_pack, sizeof(nrf_pack));
				nrf_pack.num++;
				nrf24_fifo_write(&nrf24, (uint8_t *)&nrf_pack, sizeof(nrf_pack), false);
				start_time_nrf = HAL_GetTick();
			}
		int comp;
		nrf24_irq_get(&nrf24, &comp);
		nrf24_irq_clear(&nrf24, comp);
	}

}
