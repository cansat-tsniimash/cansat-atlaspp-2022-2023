/*
 * app_main.c
 *
 *  Created on: Des 16, 2022
 *      Author: Install
 */
#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api.h>
#include <nRF24L01_PL/nrf24_defs.h>

extern SPI_HandleTypeDef hspi1;

int app_main(){
	//variables
	uint64_t tx_adrr = 0xafafafaf01;
	uint8_t arr[32] = {1, 2, 3, 4, 5};
	uint16_t packet = 123;
	nrf24_fifo_status_t tx_status;
	nrf24_fifo_status_t rx_status;
	//settings
	nrf24_spi_pins_t nrf_pins;
	nrf_pins.ce_port = GPIOA;
	nrf_pins.ce_pin = GPIO_PIN_2;
	nrf_pins.cs_port = GPIOA;
	nrf_pins.cs_pin = GPIO_PIN_3;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init(&nrf24, &hspi1, &nrf_pins);

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
	//nrf24_mode_tx(&nrf24);
	//work with fifo
	int nrf_irq;
	while(1){
		HAL_Delay(200);
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

		nrf24_irq_clear(&nrf24, nrf_irq);
		nrf24_irq_get(&nrf24, &nrf_irq);

	}
	return 0;
}
