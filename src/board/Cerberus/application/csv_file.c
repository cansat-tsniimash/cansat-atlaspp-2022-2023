/*
 * csv_file.c
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */
#include <stdio.h>
#include <string.h>
#include <structs.h>

uint16_t sd_parse_to_bytes_pack1(char *buffer, pack1_t *pack1) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%ld;%d\n",
			pack1->flag, pack1->num, pack1->time_s, pack1->accl[0], pack1->accl[1], pack1->accl[2], pack1->gyro[0], pack1->gyro[1], pack1->gyro[2], pack1->mag[0], pack1->mag[1], pack1->mag[2], pack1->bmp_temp, pack1->bmp_press, pack1->crc);
	return num_written;
}
uint16_t sd_parse_to_bytes_pack3(char *buffer, pack3_t *pack3) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%d;%d;%d;%d\n",
			pack3->flag, pack3->num, pack3->time_s, pack3->fhotorez, pack3->status, pack3->crc);
	return num_written;
}

uint16_t sd_parse_to_bytes_pack2(char *buffer, pack2_t *pack2) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%d;%d;%f;%f;%f;%d;%d\n",
			pack2->flag, pack2->num, pack2->time_s, pack2->ds_temp, pack2->lat, pack2->lon, pack2->alt, pack2->fix, pack2->crc);
	return num_written;
}

uint16_t sd_parse_to_bytes_pack4(char *buffer, pack4_t *pack4) {
	memset(buffer, 0, 300);
	uint16_t num_written = snprintf(
			buffer, 300,
			"%d;%d;%d;%ld;%ld;%d\n",
			pack4->flag, pack4->num, pack4->time_s, pack4->gps_time_s, pack4->gps_time_us, pack4->crc);
	return num_written;
}



