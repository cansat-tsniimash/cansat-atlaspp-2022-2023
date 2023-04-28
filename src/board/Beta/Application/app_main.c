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
	int8_t fix;
	uint16_t crc;
} nrf_pack_t;
#pragma pack(pop)



int app_main(){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	its_i2c_link_start();



	bus_t bus;
	bus.GPIOx = GPIOB;
	bus.GPIO_Pin = GPIO_PIN_0;
	bus.hspi = &hspi1;


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
	HAL_Delay(10);

	nrf24_spi_pins_sr_t nrf_pins;
	nrf_pins.this = &shift_reg;
	nrf_pins.pos_CE = 5;
	nrf_pins.pos_CS = 4;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init_sr(&nrf24, &hspi1, &nrf_pins);

	nrf24_rf_config_t nrf_config;
	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf_config.rf_channel = 11;
	nrf24_setup_rf(&nrf24, &nrf_config);
	nrf24_protocol_config_t nrf_protocol_config;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.en_dyn_payload_size = false;
	nrf_protocol_config.en_ack_payload = true;
	nrf_protocol_config.en_dyn_ack = false;
	nrf_protocol_config.auto_retransmit_count = 15;
	nrf_protocol_config.auto_retransmit_delay = 15;
	nrf24_setup_protocol(&nrf24, &nrf_protocol_config);
	nrf24_pipe_set_tx_addr(&nrf24, tx_adrr);
	//mods
	nrf24_mode_standby(&nrf24);
	nrf24_mode_tx(&nrf24);

	int nrf_irq;
	uint32_t start_time_nrf = HAL_GetTick();
	cmd_pack_t pack;
	nrf_pack_t nrf_pack;

	while(1){
		shift_reg_write_bit_8(&shift_reg, 7, 1);
		HAL_Delay(100);
		shift_reg_write_bit_8(&shift_reg, 7, 0);
		HAL_Delay(100);
		/*const char hello[] = "hello i'm a bus";
		int rrc = its_i2c_link_write(hello, sizeof(hello));

		int rc = its_i2c_link_read(&pack, sizeof(pack));
		if (rc > 0)
		{
			switch(pack.num){
				case CMD_1:
					if (pack.size == 1)
					{
						if (pack.data[0])
						{
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
						}
						else
						{
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

						}

					}
					break;
				case CMD_2:
					if(pack.size == 0)
					{
						mx25l512_CE(&bus);
					}
					break;
				case CMD_3:
					if ((pack.size = 5) && (pack.data[4] <= 32))
					{
						uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						uint8_t size = pack.data[4];
						mx25l512_read(&bus, &addr, pack.data, size);
						pack.size = size;
						its_i2c_link_write(&pack, sizeof(pack));

					}
					break;
				case CMD_4:
					if (pack.size >= 4 + 1)
					{
						uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						mx25l512_PP(&bus, &addr, pack.data + 4, pack.size - 4);
					}
				}
			}*/






		nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
		if (tx_status != NRF24_FIFO_FULL){
				nrf24_fifo_write(&nrf24, (uint8_t *)&nrf_pack, sizeof(&nrf_pack), false);
				start_time_nrf = HAL_GetTick();
		}
		else
		if (HAL_GetTick()-start_time_nrf >= 100)
		{
			nrf24_fifo_flush_tx(&nrf24);
			nrf24_fifo_write(&nrf24, (uint8_t *)&nrf_pack, sizeof(&nrf_pack), false);
			start_time_nrf = HAL_GetTick();
		}
	}

}
