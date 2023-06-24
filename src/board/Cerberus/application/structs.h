/*
 * structs.h
 *
 *  Created on: 5 апр. 2023 г.
 *      Author: Install
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#pragma pack(push,1) //<-------
//структурки пакетиков
// 27 byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	int16_t accl[3];
	int16_t gyro[3];
	int16_t mag[3];
	uint16_t crc;
}pack1_t;
//19byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	uint16_t bmp_temp;
	uint32_t bmp_press;
	uint16_t fhotorez;
	uint16_t status;
	uint16_t crc;
}pack3_t;
//24byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	int16_t ds_temp;
	float lat;
	float lon;
	float alt;
	int8_t fix;
	uint16_t crc;
}pack2_t;
//17 byte
typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	uint32_t gps_time_s;
	uint32_t gps_time_us;
	uint16_t crc;
}pack4_t;
typedef struct
{
	uint8_t flag;
	uint16_t num;
	uint16_t time_s;
	float lat;
	float lon;
	float alt;
	uint32_t gps_time_s;
	uint32_t gps_time_us;
	int8_t fix;
	uint16_t crc;
} nrf_pack_t;
#pragma pack(pop)

#endif /* STRUCTS_H_ */
