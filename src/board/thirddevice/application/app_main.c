/*
 * app_main.c
 *
 *  Created on: Jun 3, 2023
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
#include <ATGM336H/nmea_gps.h>
#include <string.h>
#include <bb.h>
#include <ff.h>
#define alpha_addr 0x76 << 1
#define beta_addr 0x77 << 1
#define gamma_addr 0x78 << 1

uint16_t sd_parse_to_bytes_pack(char *buffer, FRESULT is_mount, FRESULT res, int count, int count2) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%d;%d\n",
			is_mount, res, count, count2);
	return num_written;
}

int app_main(){
	FATFS fileSystem;
	FIL File_bin_alph;
	FIL File_bin_beta;
	FIL File_bin_gps;
	FIL File_bin_gam;
	FIL File;
	FRESULT res_bin_a = 255;
	FRESULT res_bin_b = 255;
	res_bin_b += 0;
	FRESULT res_bin_gps = 255;
	res_bin_gps += 0;
	FRESULT res_bin_g = 255;
	res_bin_g += 0;
	FRESULT res = 255;
	res += 0;
	const char path_bin_a[] = "packeta.bin";
	const char path_bin_b[] = "packetb.bin";
	const char path_bin_gps[] = "packetgps.bin";
	const char path_bin_g[] = "packetg.bin";
	const char path[] = "packet.csv";
	memset(&fileSystem, 0x00, sizeof(fileSystem));
	FRESULT is_mount = 0;
	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);
	UINT Bytes;
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res = f_open(&File, (char*)path, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts("is_mount; res_bin_a; count_read; count_file\n", &File);
		res = f_sync(&File);
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_a = f_open(&File_bin_alph, (char*)path_bin_a, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_b = f_open(&File_bin_beta, (char*)path_bin_b, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_gps = f_open(&File_bin_gps, (char*)path_bin_gps, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_g = f_open(&File_bin_gam, (char*)path_bin_g, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
	uint16_t str_wr;
	int rc;
	uint8_t bufa[32] = {1};
	uint8_t bufb[32] = {1};
	uint8_t bufg[32] = {1};
	uint8_t bufgps[32] = {1};
	uint16_t count = 0;
	uint16_t counter = 0;
	uint16_t count_f = 0;
	char str_buf[300];
	while(1){
		if(is_mount != 0){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, true);
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, false);
		}
		if(/*HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == */1){
			// Alpha
			count = 0;
			rc = bb_read_req(alpha_addr, 32, false);
			if (rc == HAL_OK){
				rc = bb_read(alpha_addr, (uint8_t *)bufa, sizeof(bufa));
				count++;
			} else {
				counter++;
			}
			if(res_bin_a == FR_OK){
				res_bin_a = f_write(&File_bin_alph, (uint8_t*)bufa, sizeof(bufa), &Bytes); // отправка на запись в файл
				res_bin_a = f_sync(&File_bin_alph);
				count_f++;
			}
			for(int i = 0; i < 255; i++){
				rc = bb_read_req(alpha_addr, 32, true);
				if (rc == HAL_OK){
					rc = bb_read(alpha_addr, (uint8_t *)bufa, sizeof(bufa));
					count++;
				} else {
					counter++;
				}
				if(res_bin_a == FR_OK){
					res_bin_a = f_write(&File_bin_alph, (uint8_t*)bufa, sizeof(bufa), &Bytes); // отправка на запись в файл
					res_bin_a = f_sync(&File_bin_alph);
					count_f++;
				}
			}
			str_wr = sd_parse_to_bytes_pack(str_buf, is_mount, res_bin_a, count, count_f);
			f_write(&File, str_buf, str_wr, &Bytes);
			f_sync(&File);
			count = 0;
			count_f = 0;
			// Beta
			rc = bb_read_req(beta_addr, 32, false);
			if(rc == HAL_OK){
				rc = bb_read(beta_addr, (uint8_t *)bufb, sizeof(bufb));
				count++;
			} else {
				counter++;
			}
			if(res_bin_b == FR_OK && rc == HAL_OK){
				res_bin_b = f_write(&File_bin_beta, (uint8_t*)bufb, sizeof(bufb), &Bytes); // отправка на запись в файл
				res_bin_b = f_sync(&File_bin_beta);
				count_f++;
			}
			for(int i = 0; i<255; i++){
				rc = bb_read_req(beta_addr, 32, true);
				if(rc == HAL_OK){
					rc = bb_read(beta_addr, (uint8_t *)bufb, sizeof(bufb));
					count++;
				} else {
					counter++;
				}
				if(res_bin_b == FR_OK){
					res_bin_b = f_write(&File_bin_beta, (uint8_t*)bufb, sizeof(bufb), &Bytes); // отправка на запись в файл
					res_bin_b = f_sync(&File_bin_beta);
					count_f++;
				}
			}
			str_wr = sd_parse_to_bytes_pack(str_buf, is_mount, res_bin_b, count, count_f);
			f_write(&File, str_buf, str_wr, &Bytes);
			f_sync(&File);
			count = 0;
			count_f = 0;
			//beta gps
			for(uint16_t i = 0; i<288; i++){
				uint16_t num = i;
				rc = bb_read_gps_req(beta_addr, num);
				if(rc == HAL_OK){
					rc = bb_read_gps(beta_addr, &num, (uint8_t *)bufgps, sizeof(bufgps));
					count++;
				} else {
					counter++;
				}
				if(res_bin_gps == FR_OK){
					res_bin_gps = f_write(&File_bin_gps, (uint8_t*)bufgps, sizeof(bufgps), &Bytes); // отправка на запись в файл
					res_bin_gps = f_sync(&File_bin_gps);
					count_f++;
				}
			}
			str_wr = sd_parse_to_bytes_pack(str_buf, is_mount, res_bin_gps, count, count_f);
			f_write(&File, str_buf, str_wr, &Bytes);
			f_sync(&File);
			count = 0;
			count_f = 0;
			//gamma
			rc = bb_read_req(gamma_addr, 32, false);
			if(rc == HAL_OK){
				rc = bb_read(gamma_addr, (uint8_t *)bufg, sizeof(bufg));
				count++;
			} else {
				counter++;
			}
			if(res_bin_g == FR_OK){
				res_bin_g = f_write(&File_bin_gam, (uint8_t*)bufg, sizeof(bufg), &Bytes); // отправка на запись в файл
				res_bin_g = f_sync(&File_bin_gam);
				count_f++;
			}
			for(int i = 0; i<255; i++){
				rc = bb_read_req(gamma_addr, 32, true);
				if(rc == HAL_OK){
					rc = bb_read(gamma_addr, (uint8_t *)bufg, sizeof(bufg));
					count++;
				} else {
					counter++;
				}
				if(res_bin_g == FR_OK){
					res_bin_g = f_write(&File_bin_gam, (uint8_t*)bufg, sizeof(bufg), &Bytes); // отправка на запись в файл
					res_bin_g = f_sync(&File_bin_gam);
					count_f++;
				}
			}
			str_wr = sd_parse_to_bytes_pack(str_buf, is_mount, res_bin_g, count, count_f);
			f_write(&File, str_buf, str_wr, &Bytes);
			f_sync(&File);
			count = 0;
			count_f = 0;
		}
	}
	return 0;
}


