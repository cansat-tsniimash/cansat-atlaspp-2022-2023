/*
 * app_main.c
 *
 *  Created on: Nov 12, 2022
 *      Author: Install
 */
#include <Shift_Register/shift_reg.h>
#include <stdio.h>
#include <string.h>
#include <fatfs.h>
#include <BME280/DriverForBME280.h>
#include "stm32f4xx.h"
#include <LSM6DS3/DLSM.h>
#include <LIS3MDL/DLIS3.h>
#include <Photorezistor/photorezistor.h>
#include <1Wire_DS18B20/one_wire.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_defs.h>
#include <structs.h>
#include <csv_file.h>
#include <ATGM336H/nmea_gps.h>
#include <bb.h>
#include <string.h>
#define alpha_addr 0x42 << 1
#define beta_addr 0x43 << 1
#define gamma_addr 0x44 << 1
#define devider 10
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart6;
//crc count
uint16_t Crc16(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	while (len--) {
		crc ^= *buf++ << 8;
		for (uint8_t i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}

typedef enum
{
	STATE_READY = 0,
	STATE_BEFORE_ROCKET = 1,
	STATE_IN_ROCKET = 2,
	STATE_AFTER_ROCKET = 3
} state_t;

typedef enum
{
	STATE_GEN_PACK_1_3 = 1,
	STATE_WAIT = 2,
	STATE_GEN_PACK_2_4 = 3,
} state_nrf_t;

int app_main(){
	//инициализация файла
	FATFS fileSystem; // переменная типа FATFS
	FIL File1; // хендлер файла
	FIL File2;
	FIL File3;
	FIL File4;
	FIL File_bin;
	FRESULT res1 = 255;
	FRESULT res2 = 255;
	FRESULT res3 = 255;
	FRESULT res4 = 255;
	FRESULT res_bin = 255;
	FRESULT megares = 255;
	const char path1[] = "packet1.csv";
	const char path2[] = "packet2.csv";
	const char path3[] = "packet3.csv";
	const char path4[] = "packet4.csv";
	const char path_bin[] = "packet.bin";
	memset(&fileSystem, 0x00, sizeof(fileSystem));
	FRESULT is_mount = 0;
	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res1 = f_open(&File1, (char*)path1, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts("flag; num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; crc\n", &File1);
		res1 = f_sync(&File1);
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res2 = f_open(&File2, (char*)path2, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts("flag; num; time_ms; ds_temp; lat; lon; alt; fix; crc\n", &File2);
		res2 = f_sync(&File2);
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res3 = f_open(&File3, (char*)path3, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts("flag; num; time_ms; bmp_temp; bmp_press; fhotores; status; crc\n", &File3);
		res3 = f_sync(&File3);
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res4 = f_open(&File4, (char*)path4, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts("flag; num; time_ms; gps_time_s; gps_time_us; crc\n", &File4);
		res4 = f_sync(&File4);
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin = f_open(&File_bin, (char*)path_bin, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
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
	photrez.resist = 2200;
	photrez.hadc = &hadc1;

	//структура пин оне ваера
	ds18b20_t ds;
	ds.onewire_port = GPIOA;
	ds.onewire_pin = GPIO_PIN_1;

	//Init оф ван ваер
	onewire_init(&ds);
	ds18b20_set_config(&ds, 100, -100, DS18B20_RESOLUTION_12_BIT);

	//переменные
	int alpha = false;
	int beta = false;
	int gamma = false;
	bool mount = false;

	float temperature_celsius_gyro = 0.0;
	float acc_g[3] = {0};
	float gyro_dps[3] = {0};
	float temperature_celsius_mag = 0.0;
	float mag[3] = {0};
	float lat;
	float lon;
	float alt;
	uint16_t temp_ds;
	bool crc_ok_ds = false;
	uint32_t start_time_ds = HAL_GetTick();
	uint32_t start_time_nrf = HAL_GetTick();
	uint32_t start_time_sd = HAL_GetTick();
	nrf24_fifo_status_t rx_status = NRF24_FIFO_EMPTY;
	nrf24_fifo_status_t tx_status = NRF24_FIFO_EMPTY;
	float limit_lux;
	int counter = 0;
	UINT Bytes;
	int comp = 0;
	int fast_count = 0;
	//структура бме даты
	struct bme280_data bme_data;
	bme_data = bme_read_data(&bme);
	//давление на земле
	//double ground_pressure = bme_data.pressure;

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
	nrf_config.rf_channel = 30;
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
	int64_t cookie;
	int fix_;
	shift_reg_write_bit_16(&shift_reg_n, 10, false);
	shift_reg_write_bit_16(&shift_reg_n, 11, false);
	gps_init();
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
	uint64_t gps_time_s;
	uint32_t gps_time_us;
	uint16_t str_wr;
	char str_buf[300];
	char data[] = "Hello, World!";
	gamma = bb_radio_send_d(gamma_addr, (uint8_t *)data, sizeof(data));
	shift_reg_write_bit_16(&shift_reg_n, 12, false);
	while(1){
		alpha = bb_ping(alpha_addr);
		beta = bb_ping(beta_addr);
		gamma = bb_ping(gamma_addr);

		//данные в беск цикле
		bme_data = bme_read_data(&bme);
		pack3.bmp_temp = bme_data.temperature * 100;
		pack3.bmp_press = bme_data.pressure;

		//height = 44330 * (1 - pow(pressure / ground_pressure, 1.0 / 5.255));
		float lux = photorezistor_get_lux(photrez);
		pack3.fhotorez = lux;

		lsmread(&ctx_lsm, &temperature_celsius_gyro, &acc_g, &gyro_dps);
		lisread(&ctx_lis, &temperature_celsius_mag, &mag);
		for (int i = 0; i < 3; i++){
			pack1.accl[i] = acc_g[i]*1000;
			pack1.gyro[i] = gyro_dps[i]*1000;
			pack1.mag[i] = mag[i]*1000;
		}

		if(is_mount == FR_OK){
			mount = true;
		}
		else{
			mount = false;
		}

		pack3.status = ((state_now & 0x03) << 0) |
				       ((alpha & 0x07) << 2) |
					   ((beta & 0x07) << 5) |
					   ((gamma & 0x07) << 8) |
					   ((mount & 0x01) << 11) |
				       ((comp & 0x07) << 13);

		gps_work();
		gps_get_coords(&cookie, &lat, &lon, &alt, &fix_);
		gps_get_time(&cookie, &gps_time_s, &gps_time_us);
		pack2.lat = lat;
		pack2.lon = lon;
		pack2.alt = alt;
		pack4.gps_time_s = gps_time_s;
		pack4.gps_time_us = gps_time_us;
		pack2.fix = fix_;

		//каждые 750 мс берет температуру
		if (HAL_GetTick()-start_time_ds >= 750)
		{
			ds18b20_read_raw_temperature(&ds, &temp_ds, &crc_ok_ds);
			ds18b20_start_conversion(&ds);
			start_time_ds = HAL_GetTick();
			pack2.ds_temp = ((float)temp_ds * 10) / 16;
		}

		switch (state_now)
		{
		case STATE_READY:
			//Связь с ЧЯ
			HAL_Delay(100);
			if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)){
				state_now = STATE_BEFORE_ROCKET;
				limit_lux = lux;
				shift_reg_write_bit_16(&shift_reg_n, 10, false);
				shift_reg_write_bit_16(&shift_reg_n, 11, true);
			}
			break;
		case STATE_BEFORE_ROCKET:
			//проверка признаков нахождения в ракете
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)){
				state_now = STATE_IN_ROCKET;
				limit_lux = (limit_lux - lux) * 0.8 + lux;

				alpha = bb_chip_err(alpha_addr);
				beta = bb_chip_err(beta_addr);
				gamma = bb_chip_err(gamma_addr);
				if (alpha != 0){
					shift_reg_write_bit_16(&shift_reg_n, 12, true);
					alpha = bb_buzzer_control(alpha_addr, true);
					HAL_Delay(300);
					alpha = bb_buzzer_control(alpha_addr, false);
					HAL_Delay(200);

				}
				if (gamma != 0){
					shift_reg_write_bit_16(&shift_reg_n, 12, true);
					beta = bb_buzzer_control(gamma_addr, true);
					HAL_Delay(300);
					beta = bb_buzzer_control(gamma_addr, false);
					HAL_Delay(200);
								}
				if (beta != 0){
					shift_reg_write_bit_16(&shift_reg_n, 12, true);
					beta = bb_buzzer_control(beta_addr, true);
					HAL_Delay(300);
					beta = bb_buzzer_control(beta_addr, false);
					HAL_Delay(200);
				}
				beta = bb_gps_err(beta_addr);
				if (beta != 0){
					shift_reg_write_bit_16(&shift_reg_n, 12, true);
					beta = bb_buzzer_control(beta_addr, true);
					HAL_Delay(300);
					beta = bb_buzzer_control(beta_addr, false);
					HAL_Delay(200);
				}
				shift_reg_write_bit_16(&shift_reg_n, 10, true);
				shift_reg_write_bit_16(&shift_reg_n, 11, false);

			}
			break;
		case STATE_IN_ROCKET:
			//передача данных на черные ящики и проверка фоторезистора
			if(lux >=  limit_lux){
				state_now = STATE_AFTER_ROCKET;
				shift_reg_write_bit_16(&shift_reg_n, 10, true);
				shift_reg_write_bit_16(&shift_reg_n, 11, true);
			}
			break;
		case STATE_AFTER_ROCKET:
			//запись данных на ЧЯ, передача данных на землю и запись данных на сд
			alpha = bb_buzzer_control(alpha_addr, true);
			beta = bb_buzzer_control(beta_addr, true);
			gamma = bb_buzzer_control(gamma_addr, true);
			state_now = STATE_AFTER_ROCKET;
			break;
		}


		switch(state_nrf){
		case STATE_GEN_PACK_1_3:
			pack1.time_ms = HAL_GetTick();
			pack1.num += 1;
			pack3.num += 1;
			pack1.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);
			pack3.time_ms = HAL_GetTick();
			pack3.crc = Crc16((uint8_t *)&pack3, sizeof(pack3) - 2);// <<------pack
			nrf24_fifo_flush_tx(&nrf24);
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack1, sizeof(pack1), false);//32
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack3, sizeof(pack3), false);
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack3, sizeof(pack3), false);
			fast_count++;
			if(fast_count >= 2){
				alpha = bb_write(alpha_addr, (uint8_t *)&pack1, sizeof(pack1));
				alpha = bb_write(alpha_addr, (uint8_t *)&pack3, sizeof(pack3));
				beta = bb_write(beta_addr, (uint8_t *)&pack1, sizeof(pack1));
				beta = bb_write(beta_addr, (uint8_t *)&pack3, sizeof(pack3));
				gamma = bb_write(gamma_addr, (uint8_t *)&pack1, sizeof(pack1));
				gamma = bb_write(gamma_addr, (uint8_t *)&pack3, sizeof(pack3));
				fast_count = 0;
			}
			start_time_nrf = HAL_GetTick();

			if(res1 == FR_OK){
				str_wr = sd_parse_to_bytes_pack1(str_buf, &pack1);
				res1 = f_write(&File1, str_buf, str_wr, &Bytes); // отправка на запись в файл
			}
			if(res3 == FR_OK){
				str_wr = sd_parse_to_bytes_pack3(str_buf, &pack3);
				res3 = f_write(&File3, str_buf, str_wr, &Bytes); // отправка на запись в файл
			}
			if(res3 == FR_OK){
				res_bin = f_write(&File_bin, (uint8_t*)&pack3, sizeof(pack3), &Bytes); // отправка на запись в файл
				res_bin = f_write(&File_bin, (uint8_t*)&pack1, sizeof(pack1), &Bytes); // отправка на запись в файл
			}

			state_nrf = STATE_WAIT;
			break;
		case STATE_WAIT:
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)== GPIO_PIN_RESET){
				nrf24_irq_get(&nrf24, &comp);
				nrf24_irq_clear(&nrf24, comp);
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
			pack2.time_ms = HAL_GetTick();
			pack2.num += 1;
			pack4.num += 1;
			pack2.crc = Crc16((uint8_t *)&pack2, sizeof(pack2) - 2);
			pack4.time_ms = HAL_GetTick();
			pack4.crc = Crc16((uint8_t *)&pack4, sizeof(pack4) - 2);
			nrf24_fifo_flush_tx(&nrf24);
			alpha = bb_write(alpha_addr, (uint8_t *)&pack2, sizeof(pack2));
			alpha = bb_write(alpha_addr, (uint8_t *)&pack4, sizeof(pack4));
			beta = bb_write(beta_addr, (uint8_t *)&pack2, sizeof(pack2));
			beta = bb_write(beta_addr, (uint8_t *)&pack4, sizeof(pack4));
			gamma = bb_write(gamma_addr, (uint8_t *)&pack2, sizeof(pack2));
			gamma = bb_write(gamma_addr, (uint8_t *)&pack4, sizeof(pack4));
			start_time_nrf = HAL_GetTick();
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack2, sizeof(pack2), false);
			gamma = bb_radio_send_d(gamma_addr, (uint8_t *)&pack2, sizeof(pack2));
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack4, sizeof(pack4), false);
			nrf24_fifo_write(&nrf24, (uint8_t *)&pack4, sizeof(pack4), false);

			if(res2 == FR_OK){
				str_wr = sd_parse_to_bytes_pack2(str_buf, &pack2);
				res2 = f_write(&File2, str_buf, str_wr, &Bytes); // отправка на запись в файл
			}
			if(res4 == FR_OK){
				str_wr = sd_parse_to_bytes_pack4(str_buf, &pack4);
				res4 = f_write(&File4, str_buf, str_wr, &Bytes); // отправка на запись в файл
			}
			if(res_bin == FR_OK){
				res_bin = f_write(&File_bin, (uint8_t*)&pack4, sizeof(pack4), &Bytes); // отправка на запись в файл
				res_bin = f_write(&File_bin, (uint8_t*)&pack2, sizeof(pack2), &Bytes); // отправка на запись в файл
			}
			state_nrf = STATE_WAIT;
			break;
		}


		if (HAL_GetTick()-start_time_sd >= 20)
		{
			megares = FR_OK;

			if (is_mount == FR_OK)
			{
				if(res1 != FR_OK){
					f_close(&File1);
					megares = f_open(&File1, (char*)path1, FA_WRITE | FA_OPEN_APPEND);
				}
				if(res2 != FR_OK && megares == FR_OK){
					f_close(&File2);
					megares = f_open(&File2, (char*)path1, FA_WRITE | FA_OPEN_APPEND);
				}
				if(res3 != FR_OK && megares == FR_OK){
					f_close(&File3);
					megares = f_open(&File3, (char*)path1, FA_WRITE | FA_OPEN_APPEND);
				}
				if(res4 != FR_OK && megares == FR_OK){
					f_close(&File4);
					megares = f_open(&File4, (char*)path1, FA_WRITE | FA_OPEN_APPEND);
				}
				if(res_bin != FR_OK && megares == FR_OK){
					f_close(&File_bin);
					megares = f_open(&File_bin, (char*)path_bin, FA_WRITE | FA_OPEN_APPEND);
				}
			}
			if(megares != FR_OK || is_mount != FR_OK){
				shift_reg_write_bit_16(&shift_reg_n, 9, false);
				f_mount(0, "0", 1);
				extern Disk_drvTypeDef disk;
				disk.is_initialized[0] = 0;
				is_mount = f_mount(&fileSystem, "", 1);
				res1 = f_open(&File1, (char*)path1, FA_WRITE | FA_OPEN_APPEND);
				res2 = f_open(&File2, (char*)path2, FA_WRITE | FA_OPEN_APPEND);
				res3 = f_open(&File3, (char*)path3, FA_WRITE | FA_OPEN_APPEND);
				res4 = f_open(&File4, (char*)path4, FA_WRITE | FA_OPEN_APPEND);
				res_bin = f_open(&File_bin, (char*)path_bin, FA_WRITE | FA_OPEN_APPEND);
			}
			else
			{
				shift_reg_write_bit_16(&shift_reg_n, 9, true);
				res1 = f_sync(&File1); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				res2 = f_sync(&File2); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				res3 = f_sync(&File3); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				res4 = f_sync(&File4); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				res_bin = f_sync(&File_bin); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
			}
			start_time_sd = HAL_GetTick();
		}

	}
	return 0;

}
