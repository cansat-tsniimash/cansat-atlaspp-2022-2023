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
#pragma pack(pop)


int app_main(){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	its_i2c_link_start();

	cmd_pack_t pack;

	while(1)
	{
		uint8_t * frame = (uint8_t*)&pack;
		int rc = its_i2c_link_read(&pack, sizeof(pack));
		if (rc > 0)
		{
			//printf("got frame: 0x");
			//for (int i = 0; i < rc; i++)
			//	printf("%02X", frame[i]);

			//printf("\n");

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
						//mx25l512_CE(&bus);
					}



				case CMD_3:
					if ((pack.size = 5) && (pack.data[4] <= 32)){
						uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						//mx25l512_read(&bus, pack.data, data[4]);
					}
					break;
				case CMD_4:
					if (pack.size > 4 + 1)
					{
						uint32_t addr = pack.data[0] | pack.data[1] << 8 | pack.data[2] << 16 | pack.data[3] << 24;
						//mx25l512_PP(&bus, &addr, pack.data + 4, size - 4);
					}
			}
		}








		/*if (rc > 0)
		{
			printf("rc = %d\n", rc);
			printf("0x");
			for (int i = 0; i < rc; i++)
				printf("%02X", buf[i]);

			printf("\n");

		\
	}


	/*	bus_t bus;
		bus.GPIOx = GPIOB;
		bus.GPIO_Pin = GPIO_PIN_0;
		bus.hspi = &hspi1;


		uint32_t addr = 0;
		uint8_t pData [3] = {0};
		uint8_t Data [3] = {0xFF, 0xAA, 0xCB};

	HAL_GPIO_WritePin(bus->GPIOx, bus->GPIO_Pin, GPIO_PIN_SET);

	mx25l512_wren(&bus);
	mx25l512_PP(&bus, &addr, Data, sizeof(Data));

	HAL_GPIO_WritePin(bus->GPIOx, bus->GPIO_Pin, GPIO_PIN_RESET);

	delay(100);

	HAL_GPIO_WritePin(bus->GPIOx, bus->GPIO_Pin, GPIO_PIN_SET);

	mx25l512_read(&bus, &addr, (uint8_t *)&pData, 3);
	mx25l512_wrdi(&bus);

	HAL_GPIO_WritePin(bus->GPIOx, bus->GPIO_Pin, GPIO_PIN_RESET);



 //variables
	uint64_t tx_adrr = 0xafafafaf01;
	uint8_t arr[32] = {1, 2, 3, 4, 5};
	uint16_t packet = 123;
	nrf24_fifo_status_t tx_status;
	nrf24_fifo_status_t rx_status;
//settings
	shift_reg_t shift_reg;
	shift_reg.bus = &hspi1;
	shift_reg.latch_port = GPIOA;
	shift_reg.latch_pin = GPIO_PIN_1;
	shift_reg.oe_port = GPIOA;
	shift_reg.oe_pin = GPIO_PIN_2;
	shift_reg_init(&shift_reg);

	nrf24_spi_pins_sr_t nrf_pins;
	nrf_pins.this = &shift_reg;
	nrf_pins.pos_CE = 1;
	nrf_pins.pos_CS = 2;
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
	int nrf_irq;
	while(1){
		uint8_t buf[3];
		HAL_I2C_Slave_Receive(&hi2c, buf, sizeof(buf), 10);
		cmd_t cmd = buf[0];

		switch(cmd){
			case CMD_1:
				altitude = 5;
				GPIO_Pin_speak = 1;
				break;
			case CMD_2:

			default:
		}






		nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
		if ((tx_status == NRF24_FIFO_EMPTY) || (tx_status == NRF24_FIFO_NOT_EMPTY)){
				nrf24_fifo_write(&nrf24, (uint8_t *)&packet, 32, false);
				nrf24_mode_tx(&nrf24);
				HAL_Delay(10);
				nrf24_mode_standby(&nrf24);
		}
		else if(tx_status == NRF24_FIFO_FULL){
			nrf24_fifo_flush_tx(&nrf24);
		}

		nrf24_irq_get(&nrf24, &nrf_irq);

		nrf24_irq_clear(&nrf24, nrf_irq);*/

	}

}
