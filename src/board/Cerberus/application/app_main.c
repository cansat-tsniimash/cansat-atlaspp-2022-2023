/*
 * app_main.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Install
 */
#include <Shift_Register/shift_reg.h>
#include <BME280/DriverForBME280.h>
#include "stm32f4xx.h"
#include <LSM6DS3/DLSM.h>
#include <LIS3MDL/DLIS3.h>
#include <Photorezistor/photorezistor.h>
#include <1Wire_DS18B20/one_wire.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_defs.h>

extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
//crc count
uint16_t Crc16(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	uint8_t i;
	while (len--) {
		crc ^= *buf++ << 8;
		for (i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}

typedef enum
{
	STATE_READY = 1,
	STATE_BEFORE_ROCKET = 2,
	STATE_IN_ROCKET = 3,
	STATE_AFTER_ROCKET = 4
} state_t;

typedef enum
{
	STATE_GEN_PACK_1_3 = 1,
	STATE_WAIT = 2,
	STATE_GEN_PACK_2_4 = 3,
} state_nrf_t;

#pragma pack(push,1) //<-------
//структурки пакетиков
// 31 byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint16_t time_s;
	int16_t accl[3];
	int16_t gyro[3];
	int16_t mag[3];
	uint16_t bmp_temp;
	uint32_t bmp_press;
	uint16_t crc;
}pack1_t;
//9byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint16_t time_s;
	uint16_t fhotorez;
	uint16_t status;
	uint16_t crc;
}pack3_t;
//20byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint16_t time_s;
	int16_t ds_temp;
	float lat;
	float lon;
	float alt;
	int8_t fix;
	uint16_t crc;
}pack2_t;
//13byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint16_t time_s;
	uint32_t gps_time_s;
	uint32_t gps_time_us;
	uint16_t crc;
}pack4_t;
#pragma pack(pop)

int app_main(){
	/*инициализация sr датчика*/
	shift_reg_t shift_reg_n;
	shift_reg_n.bus = &hspi2;
	shift_reg_n.latch_port = GPIOC;
	shift_reg_n.latch_pin = GPIO_PIN_1;
	shift_reg_n.oe_port = GPIOC;
	shift_reg_n.oe_pin = GPIO_PIN_13;
	shift_reg_n.value = 0;
	shift_reg_init(&shift_reg_n);
	shift_reg_write_16(&shift_reg_n, 0xFFFF);

	shift_reg_t shift_reg_r;
	shift_reg_r.bus = &hspi2;
	shift_reg_r.latch_port = GPIOC;
	shift_reg_r.latch_pin = GPIO_PIN_4;
	shift_reg_r.oe_port = GPIOC;
	shift_reg_r.oe_pin = GPIO_PIN_5;
	shift_reg_r.value = 0;
	shift_reg_init(&shift_reg_r);
	shift_reg_write_8(&shift_reg_r, 0xFF);


	/*инициализация bme*/
	struct bme_spi_intf_sr bme_struct;
	bme_struct.sr_pin = 2;
	bme_struct.sr = &shift_reg_n;
	bme_struct.spi = &hspi2;
	struct bme280_dev bme;
	bme_init_default_sr(&bme, &bme_struct);

	//стх и структура лcмa
	stmdev_ctx_t ctx_lsm;

	struct lsm_spi_intf_sr lsm_sr;
	lsm_sr.sr_pin = 4;
	lsm_sr.spi = &hspi2;
	lsm_sr.sr = &shift_reg_n;
	lsmset_sr(&ctx_lsm, &lsm_sr);

	//стх и структура лиса
	stmdev_ctx_t ctx_lis;

	struct lis_spi_intf_sr lis_sr;
	lis_sr.sr_pin = 3;
	lis_sr.spi = &hspi2;
	lis_sr.sr = &shift_reg_n;
	lisset_sr(&ctx_lis, &lis_sr);

	//структура фоторезистора
	photorezistor_t photrez;
	photrez.resist = 2000;
	photrez.hadc = &hadc1;

	//структура пин оне ваера
	ds18b20_t ds;
	ds.onewire_port = GPIOA;
	ds.onewire_pin = GPIO_PIN_1;

	//Init оф ван ваер
	onewire_init(&ds);
	ds18b20_set_config(&ds, 100, -100, DS18B20_RESOLUTION_12_BIT);

	//переменные
	float temperature_celsius_gyro = 0.0;
	float acc_g[3] = {0};
	float gyro_dps[3] = {0};
	float temperature_celsius_mag = 0.0;
	float mag[3] = {0};
	uint16_t temp_ds;
	bool crc_ok_ds = false;
	uint32_t start_time_ds = HAL_GetTick();
	uint32_t start_time_nrf = HAL_GetTick();
	nrf24_fifo_status_t rx_status = NRF24_FIFO_EMPTY;
	nrf24_fifo_status_t tx_status = NRF24_FIFO_EMPTY;
	double height = 0;
	float limit_lux;
	height += 0;
	int count = 1;
	int counter = 0;


	//структура бме даты
	struct bme280_data bme_data;
	bme_data = bme_read_data(&bme);
	//давление на земле
	double ground_pressure = bme_data.pressure;

	ds18b20_start_conversion(&ds);

	state_t state_now;
	state_now = STATE_READY;
	state_nrf_t state_nrf;
	state_nrf = STATE_GEN_PACK_1_3;



	pack1_t pack1;
	pack1.num = 0;
	pack2_t pack2;
	pack2.num = 0;
	pack3_t pack3;
	pack3.num = 0;
	pack4_t pack4;
	pack4.num = 0;

	pack1.flag = 0x21;
	pack2.flag = 0x06;
	pack3.flag = 0x20;
	pack4.flag = 0x08;

	//настройка радио
	nrf24_spi_pins_sr_t nrf_pins_sr;
	nrf_pins_sr.this = &shift_reg_r;
	nrf_pins_sr.pos_CE = 0;
	nrf_pins_sr.pos_CS = 1;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init_sr(&nrf24, &hspi2, &nrf_pins_sr);

	nrf24_mode_power_down(&nrf24);
	nrf24_rf_config_t nrf_config;
	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf_config.rf_channel = 11;
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
	nrf24_pipe_set_tx_addr(&nrf24, 0x123456789a);

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

	uint8_t buf[30] = {4, 5};
	while(1){

		HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, buf , sizeof(buf), 1);

	}

/*		//данные в беск цикле
		bme_data = bme_read_data(&bme);
		double pressure = bme_data.pressure;
		height = 44330 * (1 - pow(pressure / ground_pressure, 1.0 / 5.255));
		float lux = photorezistor_get_lux(photrez);
		lsmread(&ctx_lsm, &temperature_celsius_gyro, &acc_g, &gyro_dps);
		lisread(&ctx_lis, &temperature_celsius_mag, &mag);

		//packets

		pack1.time_s = 1;
		pack2.time_s = 1;
		pack3.time_s = 1;
		pack4.time_s = 1;
		pack1.crc = 1;
		pack2.crc = 1;
		pack3.crc = 1;
		pack4.crc = 1;

		for (int i = 0; i < 3; i++){
			pack1.accl[i] = acc_g[i];
			pack1.gyro[i] = gyro_dps[i];
			pack1.mag[i] = mag[i];
		}
		for (int i = 0; i < 3; i++)
		{
			pack1.bmp_temp = bme_data.temperature;
		}
		pack1.bmp_temp = bme_data.temperature;
		pack1.bmp_press = pressure;
		pack3.status = 1;
		pack3.fhotorez = lux;
		pack2.ds_temp = 1;
		pack2.lat = 1;
		pack2.lon = 1;
		pack2.alt = 1;
		pack2.fix = 1;
		pack4.gps_time_s = 1;
		pack4.gps_time_us = 1;

		//каждые 750 мс берет температуру
		if (HAL_GetTick()-start_time_ds >= 750)
		{
			ds18b20_read_raw_temperature(&ds, &temp_ds, &crc_ok_ds);
			ds18b20_start_conversion(&ds);
			start_time_ds = HAL_GetTick();
			temp_ds /= 16;
		}


		switch (state_now)
		{
		case STATE_READY:
			//Связь с ЧЯ
			if(1){
				state_now = STATE_BEFORE_ROCKET;
				limit_lux = lux;
			}
			break;
		case STATE_BEFORE_ROCKET:
			//проверка признаков нахождения в ракете
			if(1){
				state_now = STATE_IN_ROCKET;
				limit_lux = (limit_lux - lux) * 0.8 + lux;

			}
			break;
		case STATE_IN_ROCKET:
			//передача данных на черные ящики и проверка фоторезистора

			if(lux >=  limit_lux){
				state_now = STATE_AFTER_ROCKET;
			}
			break;
		case STATE_AFTER_ROCKET:
			//запись данных на ЧЯ, передача данных на землю и запись данных на сд

			state_now = STATE_AFTER_ROCKET;
			break;
		}
		switch(state_nrf){
		case STATE_GEN_PACK_1_3:
			pack1.time_s = HAL_GetTick();
			pack1.num += 1;
			pack3.num += 1;
			pack1.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);
			pack3.time_s = HAL_GetTick();
			pack3.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);// <<------pack
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack1, sizeof(pack1), false);//32
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack3, sizeof(pack3), false);
			start_time_nrf = HAL_GetTick();
			state_nrf = STATE_WAIT;
			break;
		case STATE_WAIT:
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)== GPIO_PIN_RESET){
				int comp;
				nrf24_irq_get(&nrf24, &comp);
				nrf24_irq_clear(&nrf24, comp);
				nrf24_irq_get(&nrf24, &comp);
				while (comp > 0)
				{
					nrf24_irq_clear(&nrf24, comp);
					nrf24_irq_get(&nrf24, &comp);
				}
				nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
				if(tx_status == NRF24_FIFO_EMPTY){
					counter++;
					if(counter == 10){
						state_nrf = STATE_GEN_PACK_2_4;
						counter = 0;
					}
					else{
						state_nrf = STATE_GEN_PACK_1_3;
					}
				}
			}
			if (HAL_GetTick()-start_time_nrf >= 100)
			{
				nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
				nrf24_fifo_flush_tx(&nrf24);
				nrf24_fifo_status(&nrf24, &rx_status, &tx_status);
				counter++;
				if(counter == 10){
					state_nrf = STATE_GEN_PACK_2_4;
					counter = 0;
				}
				else{
					state_nrf = STATE_GEN_PACK_1_3;
				}
			}
			break;
		case STATE_GEN_PACK_2_4:
			pack2.time_s = HAL_GetTick();
			pack2.num += 1;
			pack4.num += 1;
			pack2.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);
			pack4.time_s = HAL_GetTick();
			pack4.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);
			start_time_nrf = HAL_GetTick();
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack2, sizeof(pack2), false);
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack4, sizeof(pack4), false);
			state_nrf = STATE_WAIT;
			break;
		}
	}
	return 0;*/

}