/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHT3X_DIS_STM32F4XX_PORT
#define SHT3X_DIS_STM32F4XX_PORT

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_def.h"

typedef struct I2C_Handle_Port_s{
	I2C_HandleTypeDef *instance;
}I2C_Handle_Port_t;

typedef enum statusPort_e{
  STATUS_PORT_OK       = 0x00U,		//HAL_OK
  STATUS_PORT_ERROR    = 0x01U,		//HAL_ERROR
  STATUS_PORT_BUSY     = 0x02U,		//HAL_BUSY
  STATUS_PORT_TIMEOUT  = 0x03U		//HAL_TIMEOUT
} statusPort_f;

/* Functions */
statusPort_f SHT3x_DIS_write_PORT(uint16_t DevAddress, uint8_t *pData, uint16_t Size, I2C_Handle_Port_t * const i2c_handle);
statusPort_f SHT3x_DIS_read_PORT (uint16_t DevAddress, uint8_t *pData, uint16_t Size, I2C_Handle_Port_t * const i2c_handle);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif


#endif /* SHT3X_DIS_STM32F4XX_PORT */
