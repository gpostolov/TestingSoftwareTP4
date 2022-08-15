#include "unity.h"

#include "mock_stm32f4xx_hal.h"
#include "mock_stm32f4xx_hal_def.h"
#include "mock_stm32f4xx_hal_i2c.h"
#include "mock_stm32f4xx_hal_dma.h"

#include "mock_sht3x_dis_stm32F4xx_port.h"

#include "sht3x_dis.h"

//CRC TEST
#define CRC_CORRECTO   0x00
#define CRC_INCORRECTO 0x04

uint8_t data[2] = {0x00,0x00};
uint8_t nbrOfBytes = 0;
uint8_t checksum = 0x00;

void crc_set_correct (void){
    data[0] = 0xBE;
    data[1] = 0xEF;
    nbrOfBytes = 2;
    checksum = 0x92;
}

//TEST CRC INCORRECTO
void test_crc_incorrecto(void) {
    crc_set_correct();
    TEST_ASSERT_EQUAL(CRC_INCORRECTO, _SHT3x_DIS_CheckCrc( data, nbrOfBytes, checksum + 1 ));
}

//TEST CRC CORRECTO
void test_crc_correcto(void) {
    crc_set_correct();
    TEST_ASSERT_EQUAL(CRC_CORRECTO, _SHT3x_DIS_CheckCrc( data, nbrOfBytes, checksum));
}

//TEST STATUS ERROR EN _SHT3x_DIS_read_TH
void test_status_error_en_SHT3x_DIS_read_TH(void) {
    sht3x_t * const dev;
    uint32_t temp;
    uint32_t hum;

    SHT3x_DIS_read_PORT_IgnoreAndReturn(STATUS_PORT_ERROR);
    TEST_ASSERT_EQUAL(STATUS_PORT_ERROR, _SHT3x_DIS_read_TH(dev,&temp,&hum));
}

//TEST STATUS OK EN _SHT3x_DIS_read_TH
void test_status_ok_en_SHT3x_DIS_read_TH(void) {
    sht3x_t * const dev;
    uint32_t temp;
    uint32_t hum;

    SHT3x_DIS_read_PORT_IgnoreAndReturn(STATUS_PORT_OK);
    TEST_ASSERT_EQUAL(STATUS_PORT_OK, _SHT3x_DIS_read_TH(dev,&temp,&hum));
}

//TEST STATUS ERROR EN _SHT3x_DIS_write
void test_status_error_en_SHT3x_DIS_write(void) {
    sht3x_t * const dev;
    uint8_t data;

    SHT3x_DIS_write_PORT_IgnoreAndReturn(STATUS_PORT_ERROR);
    TEST_ASSERT_EQUAL(STATUS_PORT_ERROR, _SHT3x_DIS_write(dev,&data));
}

//TEST STATUS OK EN _SHT3x_DIS_write
void test_status_ok_en_SHT3x_DIS_write(void) {
    sht3x_t * const dev;
    uint8_t data;

    SHT3x_DIS_write_PORT_IgnoreAndReturn(STATUS_PORT_OK);
    TEST_ASSERT_EQUAL(STATUS_PORT_OK, _SHT3x_DIS_write(dev,&data));
}

/*
statusPort_f _SHT3x_DIS_write(sht3x_t * const dev, uint8_t *data){
	statusPort_f status_error;
	status_error = SHT3x_DIS_write_PORT(dev->address, data, 2, dev->i2c_port);
	return status_error;
}
*/

/*
//
//ESTE TEST NO ESTA BIEN ARMADO, NO LOGRO PASAR rx_buffer_exp COMPLETO!
//
//TEST DE QUE _SHT3x_DIS_read_TH CARGA VALORES DE TEMPERATURA Y HUMEDAD
void test_SHT3x_DIS_read_TH(void) {
    sht3x_t * const dev;
    uint32_t temp = 0;
    uint32_t hum  = 0;

    //Temp 25 0x6666 CRC 93
    //Hum  80 0xCCCC CRC A5
    uint8_t rx_buffer[6] = {0x00,0x00,0x0,0x00,0x00,0x00};
    uint8_t rx_buffer_exp[6] = {0x66,0x66,0x93,0xCC,0xCC,0xA5};

    statusPort_f status_error;
    
    //SHT3x_DIS_read_PORT_ExpectWithArrayAndReturn(dev->address, rx_buffer_exp, 6, 6, dev->i2c_port, 1, STATUS_PORT_OK);
    SHT3x_DIS_read_PORT_ExpectAndReturn(dev->address, rx_buffer_exp, 6, dev->i2c_port, STATUS_PORT_OK);
    SHT3x_DIS_read_PORT_IgnoreArg_pData();
    SHT3x_DIS_read_PORT_ReturnThruPtr_pData(rx_buffer_exp);

    status_error = _SHT3x_DIS_read_TH(dev,&temp,&hum);

    TEST_ASSERT_EQUAL(STATUS_PORT_OK, status_error);
    //TEST_ASSERT_EQUAL(25, temp);
    //TEST_ASSERT_EQUAL(80, hum);
}
*/