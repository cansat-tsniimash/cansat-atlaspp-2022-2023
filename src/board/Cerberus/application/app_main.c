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

extern SPI_HandleTypeDef hspi2;

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

	struct lsm_spi_intf_sr lsm_sr;
	lsm_sr.sr_pin = 4;
	lsm_sr.spi = &hspi2;
	lsm_sr.sr = &shift_reg_n;
	stmdev_ctx_t ctx;
	lsmset_sr(&ctx, &lsm_sr);

	float temperature_celsius_gyro = 0.0;
	float acc_g[3] = {0};
	float gyro_dps[3] = {0};

	HAL_Delay(100);
	struct bme280_data bme_data;
	bme_data = bme_read_data(&bme);
	double ground_pressure = bme_data.pressure;

	while(1){
		bme_data = bme_read_data(&bme);
		double pressure = bme_data.pressure;
		double height = 44330 * (1 - pow(pressure / ground_pressure, 1.0 / 5.255));
		lsmread(&ctx, &temperature_celsius_gyro, &acc_g, &gyro_dps);
	}
	return 0;
}
