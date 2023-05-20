#include <stdint.h>
#include <stm32f0xx_hal.h>

int _write(int file, char *ptr, int len)
{
	extern UART_HandleTypeDef huart2;
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
