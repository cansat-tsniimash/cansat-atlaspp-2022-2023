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
#include <mem-proxy.h>
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

uint8_t buf[32] = {0x12, 0x34, 0x56};

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
	shift_reg_write_bit_8(shift_reg, 3, true);
}

void on_bb(shift_reg_t *shift_reg){
	shift_reg_write_bit_8(shift_reg, 3, false);
}

void buzzer_control(shift_reg_t *shift_reg, bool onoff){
	shift_reg_write_bit_8(shift_reg, 7, onoff);
}

typedef enum
{
	SYS_TEST_MEM_DATA = (1 << 0),
	SYS_TEST_NRF24 = (1 << 1),
	SYS_TEST_MEM_GPS = (1 << 2)
} sys_check_t;

int all_systems_check(nrf24_lower_api_config_t *nrf24, bus_t *bus_data, bus_t *bus_gps, int comp)
{
	int ret = 0;

	if (comp & SYS_TEST_NRF24)
	{
		int irq;
		nrf24_fifo_status_t rx_status;
		nrf24_fifo_status_t tx_status;
		nrf24_fifo_flush_tx(nrf24);
		nrf24_fifo_flush_rx(nrf24);
		nrf24_fifo_status(nrf24, &rx_status, &tx_status);
		nrf24_irq_get(nrf24, &irq);
		if ((irq < 6) && (rx_status == NRF24_FIFO_EMPTY) && (tx_status == NRF24_FIFO_EMPTY))
			ret |= SYS_TEST_NRF24;
	}
	if (comp & SYS_TEST_MEM_DATA)
	{
		uint8_t id[3] = {0};
		mx25l512_rdid(bus_data, id);
		if ((id[0] == 0xc2) && (id[1] == 0x20) && (id[2] == 0x10))
			ret |= SYS_TEST_MEM_DATA;
	}
	if (comp & SYS_TEST_MEM_GPS)
	{
		uint8_t id[3] = {0};
		mx25l512_rdid(bus_gps, id);
		if ((id[0] == 0xc2) && (id[1] == 0x20) && (id[2] == 0x10))
			ret |= SYS_TEST_MEM_GPS;
	}
	return ret;
}

uint8_t read_buf[256];

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
	shift_reg_write_8(&shift_reg, 0xF0);
	on_bb(&shift_reg);
	buzzer_control(&shift_reg, false);
	shift_reg_oe(&shift_reg, false);


	bus_t bus_data;
	mx25l512_spi_pins_sr_t mx25_data_pins;
	mx25_data_pins.this = &shift_reg;
	mx25_data_pins.pos_CS = 6;
	mx25l512_spi_init_sr(&bus_data, &hspi1, &mx25_data_pins);

	uint8_t byte_r = 0;
	uint8_t byte_w = 74;
	uint8_t stat_reg = 0;

	uint32_t addr2 = 8;

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
	nrf_config.rf_channel = 40;
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
	nrf24_pipe_set_tx_addr(&nrf24, 0x143456789a);

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

	int check = all_systems_check(&nrf24, &bus_data, &bus_data, SYS_TEST_NRF24 | SYS_TEST_MEM_DATA);
	buzzer_control(&shift_reg, true);
	HAL_Delay(500);
	buzzer_control(&shift_reg, false);
	HAL_Delay(300);
	for (int i = 0; i < check; i++)
	{
		buzzer_control(&shift_reg, true);
		HAL_Delay(300);
		buzzer_control(&shift_reg, false);
		HAL_Delay(300);
	}

	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);

	uint32_t addr_write = 0;
	uint32_t addr_read = 0;
	int nrf_irq;
	uint32_t addr = 0x00;
	settings_pack_t settings_pack;
	uint32_t start_time_nrf = HAL_GetTick();
	uint32_t time_nrf = HAL_GetTick();
	cmd_pack_t pack;
	nrf_pack_t nrf_pack = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};


	memproxy_init(&bus_data);


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
				period = 1000;
				check_i = 0;
			}
			else
			{
				period = 150;
				check_i++;
			}

		}
		a++;

		int rc = its_i2c_link_read(&pack, sizeof(pack));
		/*for (int i = 0 ; i < 100; i++)
			{
				volatile uint8_t buffer[200];
				memset(buffer, i, 200);
				memproxy_write(buffer, 200);
				volatile int x = 0;
				//HAL_Delay(100);
			}

			for (int i = 0 ; i < 100; i++)
			{
				volatile uint8_t buffer[0x100];
				memset(buffer, 0xcc, 0x100);
				uint32_t addr = 0x100*i;
				mx25l512_read(&bus_data, &addr, buffer, 0x100);

				volatile int x = 0;
			}*/
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
						uint32_t addr = (pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24) & 0xFF00;
						uint8_t size = pack.data[4];
						mx25l512_read(&bus_data, &addr, read_buf, 256);//читаю данные
						memcpy(pack.data + 4, read_buf + ((pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24) & 0xFF), size);
						pack.size = size + 4;
						its_i2c_link_write(&pack, sizeof(pack));
					}
					/*if ((pack.size = 5) && (pack.data[4] <= 32))
					{
						uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						uint8_t size = pack.data[4];
						mx25l512_read(&bus_data, &addr, pack.data + 4, size);//читаю данные
						pack.size = size + 4;
						its_i2c_link_write(&pack, sizeof(pack));
					}*/
					break;
				case CMD_Write:
					if (pack.size <= 32)
					{
						memproxy_write(pack.data, pack.size);
/*						uint8_t size = pack.size;
						uint32_t new_addr = addr_write + size;

						if(new_addr < 0xffff){
							if ((addr_write & (0x0f << 12)) != (new_addr & (0x0f << 12)))
							{
								addr_write = new_addr & (0x0f << 12);
								new_addr = addr_write + size;
							}
							mx25l512_PP_up(&bus_data, &addr_write, pack.data, pack.size, 10);//Записываю данные
							addr_write = new_addr;
							pack.size = size;
							its_i2c_link_write(&pack, sizeof(pack));
						}*/
					}
					break;
				case CMD_Read:
					if (pack.size == 1 && pack.data[0] <= 32)
					{
						addr_read = 0x0000;
						uint8_t size = pack.data[0];
						mx25l512_read(&bus_data, &addr_read, read_buf, 256);//читаю данные
						memcpy(pack.data, read_buf, size);
						addr_read = size;
						pack.size = size;
						its_i2c_link_write(&pack, sizeof(pack));

					}
					/*if (pack.size == 1 && pack.data[0] <= 32)
					{
						addr_read = 0x0000;
						uint8_t size = pack.data[0];
						mx25l512_read(&bus_data, &addr_read, pack.data, size);
						addr_read = size << 4;
						pack.size = size;
						its_i2c_link_write(&pack, sizeof(pack));

					}*/
					break;
				case CMD_Continue:
					if (pack.size == 1 && pack.data[0] <= 32)
					{
						uint8_t size = pack.data[0];
						if(addr_read + size < 0xffff){
							uint32_t new_addr = addr_read & 0xFF00;
							mx25l512_read(&bus_data, &new_addr, read_buf, 256);//читаю данные
							if (((addr_read & 0xFF) + size) > 0xFF)
							{
								uint8_t part = 0xFF - (addr_read & 0xFF) + 1;
								memcpy(pack.data, read_buf + (addr_read & 0xFF), part);
								new_addr = (addr_read + size) & 0xFF00;
								mx25l512_read(&bus_data, &new_addr, read_buf, 256);
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
					/*if (pack.size == 1 && pack.data[0] <= 32)
					{
						uint8_t size = pack.data[0];
						uint32_t new_addr = addr_read + size;
						if(new_addr < 0xffff){
							if ((addr_read & (0x0f << 12)) != (new_addr & (0x0f << 12)))
							{
								addr_read = new_addr & (0x0f << 12);
								new_addr = addr_read + size;
							}
							mx25l512_read(&bus_data, &addr_read, pack.data, size);
							addr_read = new_addr;
							pack.size = size;
							its_i2c_link_write(&pack, sizeof(pack));
						}
					}*/
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

		int comp;
		nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
		if (tx_status != NRF24_FIFO_FULL)
		{
			if (HAL_GetTick() - time_nrf >= 1){
				nrf24_fifo_write(&nrf24, (uint8_t *)buf, sizeof(buf), false);
				time_nrf = HAL_GetTick();
			}
		}
		else
		if (HAL_GetTick()-start_time_nrf >= 100)
		{
			nrf24_fifo_flush_tx(&nrf24);
			nrf24_fifo_write(&nrf24, (uint8_t *)buf, sizeof(buf), false);
			start_time_nrf = HAL_GetTick();
		}
		nrf24_irq_get(&nrf24, &comp);
		nrf24_irq_clear(&nrf24, comp);
	}

}
