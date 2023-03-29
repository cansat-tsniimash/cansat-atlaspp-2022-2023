/*
 * bb.c
 *
 *  Created on: Mar 29, 2023
 *      Author: Install
 */

#include "bb.h"

#pragma pack(push,1)
typedef struct
{
    uint8_t num;
    uint8_t size;
    uint8_t data[36];
} i2c_pack_t;
#pragma pack(pop)

extern I2C_HandleTypeDef hi2c1;

int bb_buzzer_control(bool onoff)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_1;
	i2c_pack.size = 1;
	i2c_pack.data[0] = onoff;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, &cmd, sizeof(cmd), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, (uint8_t *)&i2c_pack, sizeof(i2c_pack), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}

int bb_chip_err()
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_2;
	i2c_pack.size = 0;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, &cmd, sizeof(cmd), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, (uint8_t *)&i2c_pack, sizeof(i2c_pack), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}



//void bb_read(uint32_t addr, uint8_t* buf, uint8_t size)
//{
/**
 uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_3;
	i2c_pack.size = 5;
	memcpy(i2c_pack.data, &addr, 4);
	if (size > 32)
		data[4] = 32;
	else
		data[4] = size;
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, &cmd, sizeof(cmd), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, (uint8_t *)&i2c_pack, sizeof(i2c_pack), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	return rc;

	HAL_Delay(10);

 */
//	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
//	i2c_pack_t i2c_pack;
//}

int bb_write(uint32_t addr, uint8_t* buf, uint8_t size)
{
	uint8_t cmd = I2C_LINK_CMD_SET_PACKET;
	i2c_pack_t i2c_pack;
	i2c_pack.num = CMD_4;
	if (size > 32){
		i2c_pack.size = 32;
	}
	else{
		i2c_pack.size = size + 4;
	}
	memcpy(i2c_pack.data, &addr, 4);
	memcpy(i2c_pack.data + 4, buf, i2c_pack.size-4);
	HAL_StatusTypeDef rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, &cmd, sizeof(cmd), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	if (rc != HAL_OK)
	{
		return rc;
	}
	rc = HAL_I2C_Master_Transmit(&hi2c1, 0x77<<1, (uint8_t *)&i2c_pack, sizeof(i2c_pack), 1);
	if (rc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 100);
	}
	return rc;
}
