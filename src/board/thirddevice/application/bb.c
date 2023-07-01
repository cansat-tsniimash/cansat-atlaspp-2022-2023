/*
 * bb.c
 *
 *  Created on: Mar 29, 2023
 *      Author: Install
 */

#include "bb.h"
#include <stdio.h>
#include "main.h"
#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api.h>
#include <nRF24L01_PL/nrf24_defs.h>
#pragma pack(push,1)
typedef struct
{
    uint8_t num;
    uint8_t size;
    uint8_t data[36];
} i2c_pack_t;
#pragma pack(pop)

extern I2C_HandleTypeDef hi2c1;

#define I2C_TIMEOUT 40

int bb_buzzer_control(uint16_t I2C_ADDRES, bool onoff)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_BUZ;
	i2c_pack.size = 1;
	i2c_pack.data[0] = onoff;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_chip_err(uint16_t I2C_ADDRES)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_CE;
	i2c_pack.size = 0;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_gps_err(uint16_t I2C_ADDRES)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_CE_GPS;
	i2c_pack.size = 0;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}
int bb_read_req_addr(uint16_t I2C_ADDRES, uint32_t addr, uint8_t size){
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_ReadADDR;
	i2c_pack.size = 5;
	memcpy(i2c_pack.data, &addr, 4);
	if (size > 32)
	 	i2c_pack.data[4] = 32;
	else
	 	i2c_pack.data[4] = size;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	return rc;
}

int bb_read_addr(uint16_t I2C_ADDRES, uint32_t *addr, uint8_t* buf, uint8_t size)
{
 	uint16_t size_mes = 0;
	i2c_pack_t i2c_pack;
	uint8_t cmd = I2C_LINK_CMD_GET_SIZE;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, 1, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	HAL_Delay(10);
	rc = HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRES, (uint8_t *)&size_mes, 2, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	if(size_mes <= 0 || size_mes > 42){
		return 4;
	}
	cmd = I2C_LINK_CMD_GET_PACKET;
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, 1, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	rc = HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	memcpy(addr, i2c_pack.data, 4);
	if(i2c_pack.size - 4 <= size)
		memcpy(buf, i2c_pack.data + 4, i2c_pack.size - 4);
	if(i2c_pack.size - 4 > size)
		memcpy(buf, i2c_pack.data + 4, size);
	return rc;
}

int bb_read_req(uint16_t I2C_ADDRES, uint8_t size, bool is_continue){
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	if(!is_continue)
		i2c_pack.num = CMD_Read;
	else
		i2c_pack.num = CMD_Continue;
	i2c_pack.size = 1;
	if (size > 32)
	 	i2c_pack.data[0] = 32;
	else
	 	i2c_pack.data[0] = size;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY || rc == HAL_ERROR)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	return rc;
}

int bb_read(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size)
{
 	uint16_t size_mes = 0;
	i2c_pack_t i2c_pack;
	uint8_t cmd = I2C_LINK_CMD_GET_SIZE;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, 1, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	HAL_Delay(10);
	rc = HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRES, (uint8_t *)&size_mes, 2, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	if(size_mes <= 0 || size_mes > 42){
		return 4;
	}
	cmd = I2C_LINK_CMD_GET_PACKET;
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, 1, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	rc = HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	if(i2c_pack.size <= size)
		memcpy(buf, i2c_pack.data, i2c_pack.size);
	if(i2c_pack.size > size)
		memcpy(buf, i2c_pack.data, size);
	return rc;
}


int bb_write(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_Write;
	if (size > 32){
		i2c_pack.size = 32;
	}
	else{
		i2c_pack.size = size;
	}
	memcpy(i2c_pack.data, buf, i2c_pack.size);
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}
int bb_off(uint16_t I2C_ADDRES){
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_OFF;
	i2c_pack.size = 0;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_read_gps_req(uint16_t I2C_ADDRES, uint16_t num){
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_Read_gps;
	i2c_pack.size = 2;
	memcpy(i2c_pack.data, &num, 2);
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	return rc;
}

int bb_read_gps(uint16_t I2C_ADDRES, uint16_t *num, uint8_t* buf, uint8_t size)
{
 	uint16_t size_mes = 0;
	i2c_pack_t i2c_pack;
	uint8_t cmd = I2C_LINK_CMD_GET_SIZE;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, 1, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	HAL_Delay(10);
	rc = HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRES, (uint8_t *)&size_mes, 2, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	if(size_mes == 32){
		return 4;
	}
	cmd = I2C_LINK_CMD_GET_PACKET;
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, 1, I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	rc = HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
	 	I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
	 	return rc;
	}
	memcpy(num, i2c_pack.data, 2);
	if(i2c_pack.size - 2 <= size)
		memcpy(buf, i2c_pack.data + 2, i2c_pack.size);
	if(i2c_pack.size - 2 > size)
		memcpy(buf, i2c_pack.data + 2, size);
	return rc;
}

int bb_write_flys_bit(uint16_t I2C_ADDRES, bool onoff)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_Write_flys_bit;
	i2c_pack.size = 1;
	i2c_pack.data[0] = onoff;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_radio_send(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size){
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_Radio_send;
	if (size > 32){
		i2c_pack.size = 32;
	}
	else{
		i2c_pack.size = size;
	}
	memcpy(i2c_pack.data, buf, i2c_pack.size);
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_radio_send_d(uint16_t I2C_ADDRES, uint8_t* buf, uint8_t size){
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_Radio_send_d;
	if (size > 32){
		i2c_pack.size = 32;
	}
	else{
		i2c_pack.size = size;
	}
	memcpy(i2c_pack.data, buf, i2c_pack.size);
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_settings_pack(uint16_t I2C_ADDRES, settings_pack_t *settings_pack)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_Settings;
    i2c_pack.size = 18;

	memcpy(i2c_pack.data, settings_pack, i2c_pack.size);
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, &cmd, sizeof(cmd), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRES, (uint8_t *)&i2c_pack, sizeof(i2c_pack), I2C_TIMEOUT);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}





















