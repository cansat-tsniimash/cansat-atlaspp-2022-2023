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

extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;

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
	shift_reg_push_16(&shift_reg_n, 0xFFFF);


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

	//структура бме даты
	struct bme280_data bme_data;
	bme_data = bme_read_data(&bme);
	//давление на земле
	double ground_pressure = bme_data.pressure;

	ds18b20_start_conversion(&ds);

	while(1){
		//данные в беск цикле
		bme_data = bme_read_data(&bme);
		double pressure = bme_data.pressure;
		double height = 44330 * (1 - pow(pressure / ground_pressure, 1.0 / 5.255));
		float lux = photorezistor_get_lux(photrez);
		lsmread(&ctx_lsm, &temperature_celsius_gyro, &acc_g, &gyro_dps);
		lisread(&ctx_lis, &temperature_celsius_mag, &mag);
		//каждые 750 мс берет температуру
		if (HAL_GetTick()-start_time_ds >= 750)
		{
			ds18b20_read_raw_temperature(&ds, &temp_ds, &crc_ok_ds);
			ds18b20_start_conversion(&ds);
			start_time_ds = HAL_GetTick();
			temp_ds /= 16;
		}
	}
	return 0;
}
