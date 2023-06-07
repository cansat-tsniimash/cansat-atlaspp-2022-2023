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

int app_main(){
	FATFS fileSystem;
	FIL File_bin_alph;
	FIL File_bin_beta;
	FIL File_bin_gam;
	FIL File;
	FRESULT res_bin_a = 255;
	FRESULT res_bin_b = 255;
	FRESULT res_bin_g = 255;
	FRESULT res = 255;
	const char path_bin_a[] = "packeta.bin";
	const char path_bin_b[] = "packetb.bin";
	const char path_bin_g[] = "packetg.bin";
	const char path[] = "packet.bin";
	memset(&fileSystem, 0x00, sizeof(fileSystem));
	FRESULT is_mount = 0;
	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);
	UINT Bytes;
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res = f_open(&File, (char*)path, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
		f_puts(" ", &File);
		res = f_sync(&File);
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_a = f_open(&File_bin_alph, (char*)path_bin_a, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_b = f_open(&File_bin_beta, (char*)path_bin_b, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res_bin_g = f_open(&File_bin_gam, (char*)path_bin_g, FA_WRITE | FA_CREATE_ALWAYS); // открытие файла, обязательно для работы с ним
	}

	int rc;
	uint32_t buf[32] = {1};

	while(1){
		if(/*HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == */1){
			rc = bb_read_req(alpha_addr, 32, false);
			rc = bb_read(alpha_addr, (uint8_t *)buf, sizeof(buf));
			if(rc != 0){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, true);
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, false);
			}
			if(res_bin_a == FR_OK){
				res_bin_a = f_write(&File_bin_alph, (uint8_t*)buf, sizeof(buf), &Bytes); // отправка на запись в файл
				res_bin_a = f_sync(&File_bin_alph);
			}
			for(int i = 0; i<255; i++){
				bb_read_req(alpha_addr, 32, true);
				bb_read(alpha_addr, (uint8_t *)buf, sizeof(buf));
				if(res_bin_a == FR_OK){
					res_bin_a = f_write(&File_bin_alph, (uint8_t*)buf, sizeof(buf), &Bytes); // отправка на запись в файл
					res_bin_a = f_sync(&File_bin_alph);
				}
			}
		}
	}
	return 0;
}


