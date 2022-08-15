#include "sht3x_dis_stm32F4xx_port.h"

#define TIMEOUT 2000

statusPort_f SHT3x_DIS_write_PORT(uint16_t DevAddress, uint8_t *pData, uint16_t Size, I2C_Handle_Port_t * const i2c_handle){
	HAL_StatusTypeDef i2c_status;
	i2c_status = HAL_I2C_Master_Transmit(i2c_handle->instance, DevAddress<<1, pData, Size,TIMEOUT);
	return (statusPort_f)i2c_status;
}

statusPort_f SHT3x_DIS_read_PORT(uint16_t DevAddress, uint8_t *pData, uint16_t Size, I2C_Handle_Port_t * const i2c_handle){
	HAL_StatusTypeDef i2c_status;
	HAL_Delay(10);
	i2c_status = HAL_I2C_Master_Receive(i2c_handle->instance, (DevAddress<<1), pData, Size,TIMEOUT);
	return (statusPort_f)i2c_status;
}
