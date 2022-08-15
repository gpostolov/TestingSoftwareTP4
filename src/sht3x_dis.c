/**
 ******************************************************************************
 * @file      SHT3x_DIS.c
 * @author    Driver para el SHT3x-DIS
 * @brief
 ******************************************************************************
 */

/* Includes */
#include <stdio.h>
#include "sht3x_dis.h"

/* Defines */
#define MAX_LENGHT 4


/* Variables */
I2C_Handle_Port_t i2c;

/* Functions */
statusPort_f SHT3x_DIS_init(sht3x_t * const dev, I2C_Handle_Port_t* i2c){
	if (dev == NULL ) return STATUS_PORT_ERROR;
	if (i2c == NULL ) return STATUS_PORT_ERROR;
	dev->i2c_port = i2c;
	return STATUS_PORT_OK;
}

statusPort_f SHT3x_DIS_config(sht3x_t * const dev,  sht3x_address_t my_address, sht3x_mode_t my_mode){
	if (dev == NULL ) return STATUS_PORT_ERROR;

	//Address
	if ((my_address == SHT3X_ADDRESS_A) || (my_address == SHT3X_ADDRESS_B)){
		dev->address = my_address;
	}else{
		dev->address = SHT3X_ADDRESS_A;
	}

	//Mode
	switch (my_mode){
	//Single Shot
	case SHT3X_MODE_HIGH_CS_ENABLED:
	case SHT3X_MODE_MEDIUM_CS_ENABLED:
	case SHT3X_MODE_LOW_CS_ENABLED:
	case SHT3X_MODE_HIGH_CS_DISABLED:
	case SHT3X_MODE_MEDIUM_CS_DISABLED:
	case SHT3X_MODE_LOW_CS_DISABLED:
	//Periodic
	case SHT3X_MODE_HIGH_MPS005:
	case SHT3X_MODE_MEDIUM_MPS005:
	case SHT3X_MODE_LOW_MPS005:
	case SHT3X_MODE_HIGH_MPS010:
	case SHT3X_MODE_MEDIUM_MPS010:
	case SHT3X_MODE_LOW_MPS010:
	case SHT3X_MODE_HIGH_MPS020:
	case SHT3X_MODE_MEDIUM_MPS020:
	case SHT3X_MODE_LOW_MPS020:
	case SHT3X_MODE_HIGH_MPS040:
	case SHT3X_MODE_MEDIUM_MPS040:
	case SHT3X_MODE_LOW_MPS040:
	case SHT3X_MODE_HIGH_MPS100:
	case SHT3X_MODE_MEDIUM_MPS100:
	case SHT3X_MODE_LOW_MPS100:
		dev->mode = my_mode;
		break;
	default:
		dev->mode = SHT3X_MODE_HIGH_CS_ENABLED;
		break;
	}
	return STATUS_PORT_OK;
}

statusPort_f SHT3x_DIS_reset(sht3x_t * const dev){
	statusPort_f status_error;
	if (dev == NULL ) return STATUS_PORT_ERROR;
	uint8_t tx_buffer[2];
	tx_buffer[0] = (SHT3X_COMMAND_SOFT_RESET & 0xFF00u) >> 8u;
	tx_buffer[1] =  SHT3X_COMMAND_SOFT_RESET & 0x00FFu;
	status_error = _SHT3x_DIS_write(dev,tx_buffer);
	return status_error;
}

statusPort_f SHT3x_DIS_heater(sht3x_t * const dev, bool enabled){
	statusPort_f status_error;
	if (dev == NULL ) return STATUS_PORT_ERROR;

	uint8_t tx_buffer[2];
	if(enabled){
		tx_buffer[0] = (SHT3X_COMMAND_HEATER_ENABLED & 0xFF00u) >> 8u;
		tx_buffer[1] =  SHT3X_COMMAND_HEATER_ENABLED & 0x00FFu;
	}else{
		tx_buffer[0] = (SHT3X_COMMAND_HEATER_DISABLED & 0xFF00u) >> 8u;
		tx_buffer[1] =  SHT3X_COMMAND_HEATER_DISABLED & 0x00FFu;
	}
	status_error = _SHT3x_DIS_write(dev,tx_buffer);
	return status_error;
}

statusPort_f SHT3x_DIS_read_status(sht3x_t * const dev, uint8_t *status){
	statusPort_f status_error;
	if (dev == NULL ) return STATUS_PORT_ERROR;
	if (status == NULL ) return STATUS_PORT_ERROR;
	uint8_t tx_buffer[2];
	tx_buffer[0] = (SHT3X_COMMAND_READ_STATUS & 0xFF00u) >> 8u;
	tx_buffer[1] =  SHT3X_COMMAND_READ_STATUS & 0x00FFu;
	status_error = _SHT3x_DIS_write(dev,tx_buffer);
	if(status_error != STATUS_PORT_OK) return status_error;
	status_error = _SHT3x_DIS_read_STATUS(dev,status);
	return status_error;
}

statusPort_f SHT3x_DIS_clear_status(sht3x_t * const dev){
	statusPort_f status_error;
	if (dev == NULL ) return STATUS_PORT_ERROR;
	uint8_t tx_buffer[2];
	tx_buffer[0] = (SHT3X_COMMAND_CLEAR_STATUS & 0xFF00u) >> 8u;
	tx_buffer[1] =  SHT3X_COMMAND_CLEAR_STATUS & 0x00FFu;
	status_error = _SHT3x_DIS_write(dev,tx_buffer);
	return status_error;
}

statusPort_f SHT3x_DIS_read_TH(sht3x_t * const dev, uint32_t *temp,uint32_t *hum){
	statusPort_f status_error;
	if (dev == NULL ) return STATUS_PORT_ERROR;
	if (temp == NULL ) return STATUS_PORT_ERROR;
	if (hum == NULL ) return STATUS_PORT_ERROR;

	uint8_t tx_buffer[2];
	tx_buffer[0] = ((dev->mode) & 0xFF00u) >> 8u;
	tx_buffer[1] =  (dev->mode) & 0x00FFu;
	status_error = _SHT3x_DIS_write(dev, tx_buffer);
	if(status_error != STATUS_PORT_OK) return status_error;
	status_error = _SHT3x_DIS_read_TH(dev, temp, hum);
	return status_error;
}

statusPort_f _SHT3x_DIS_write(sht3x_t * const dev, uint8_t *data){
	statusPort_f status_error;
	status_error = SHT3x_DIS_write_PORT(dev->address, data, 2, dev->i2c_port);
	return status_error;
}

statusPort_f _SHT3x_DIS_read_TH(sht3x_t * const dev, uint32_t *temp,uint32_t *hum){
	
	statusPort_f status_error;
	uint8_t rx_buffer[6];

	rx_buffer[0] = 0x00;
	rx_buffer[1] = 0x00;
	rx_buffer[2] = 0x00;
	rx_buffer[3] = 0x00;
	rx_buffer[4] = 0x00;
	rx_buffer[5] = 0x00;

	status_error = SHT3x_DIS_read_PORT(dev->address, rx_buffer,6,dev->i2c_port);

	if(status_error != STATUS_PORT_OK) return status_error;

	//CheckCrc
	if (_SHT3x_DIS_CheckCrc(&rx_buffer[0], 2, rx_buffer[2]) == 0){
		*temp = (uint32_t)(((rx_buffer[0]*256) + rx_buffer[1])*175)/65535.0-45.0;
	} else {
		//*temp = 255; //Error
		*temp = rx_buffer[0]; //Error
	}

	//CheckCrc
	if (_SHT3x_DIS_CheckCrc(&rx_buffer[3], 2, rx_buffer[5]) == 0){
		*hum =  (uint32_t)(((rx_buffer[3]*256) + rx_buffer[4]))*100.0/65535.0;
	} else {
		//*hum = 255; //Error
		*hum = rx_buffer[5]; //Error
	}

	return STATUS_PORT_OK;
}

statusPort_f _SHT3x_DIS_read_STATUS(sht3x_t * const dev, uint8_t *status){
	statusPort_f status_error;
	uint8_t rx_buffer[3];

	rx_buffer[0] = 0x00;
	rx_buffer[1] = 0x00;
	rx_buffer[2] = 0x00;

	status_error = SHT3x_DIS_read_PORT(dev->address, rx_buffer,3,dev->i2c_port);
	if(status_error != STATUS_PORT_OK) return status_error;

	//CheckCrc
	if (_SHT3x_DIS_CheckCrc(&rx_buffer[0], 2, rx_buffer[2]) == 0){
		status[0] = rx_buffer[0];
		status[1] = rx_buffer[1];
	}

	return STATUS_PORT_OK;
}

uint8_t _SHT3x_DIS_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum){
	uint8_t crc = 0xff;
	uint8_t byteCtr;
	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
		crc ^= (data[byteCtr]);
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80)
				crc = (uint8_t)((uint8_t)(crc << 1) ^ POLYNOMIAL);
			else
				crc = (crc << 1);
		}
	}
	if (crc != checksum)
		return CHECKSUM_ERROR;
	else
		return 0;
}
