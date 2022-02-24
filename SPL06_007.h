#ifndef SPL06_007_H
#define SPL06_007_H

#include "main.h"

/* -------
*
*
* Defines
*
*
*
------- */

/* -------
* REGISTER ADDRESSES
-------- */

// Device ADDR (I2C)
#define SPL_ADDR_W								0x76 << 1
#define SPL_ADDR_R								0x76 << 1 | 1

// Pressure regs
#define SPL_PSR_B2								0x00
#define SPL_PSR_B1								0x01
#define SPL_PSR_B0								0x02

// Temperature regs
#define SPL_TMP_B2								0x03
#define SPL_TMP_B1								0x04
#define SPL_TMP_B0								0x05

// Configuration regs
#define SPL_PRS_CFG								0x06
#define SPL_TMP_CFG								0x07
#define SPL_MEAS_CFG							0x08
#define SPL_CFG_REG								0x09

// Status regs
#define SPL_INT_STS								0x0A
#define SPL_FIFO_STS							0x0B

// Reset
#define SPL_RESET								0x0C

// ID
#define SPL_ID									0x0D

// Adjustment coeffs
#define SPL_COEF								0x10
#define SPL_COEF_LEN							18



/* -------
* REGISTER DEFINES
-------- */

/* ********************* */
// PRS_CFG
// Pressure configuration register
// Addr: 0x06
// Default: 0x00

// PM_RATE - Pressure measurement rate
// For background mode
#define SPL_PM_RATE_1							0x00 << 4
#define SPL_PM_RATE_2							0x01 << 4
#define SPL_PM_RATE_4							0x02 << 4
#define SPL_PM_RATE_8							0x03 << 4
#define SPL_PM_RATE_16							0x04 << 4
#define SPL_PM_RATE_32							0x05 << 4
#define SPL_PM_RATE_64							0x06 << 4
#define SPL_PM_RATE_128							0x07 << 4

// PM_PRC - Pressure oversampling rate
#define SPL_PM_PRC_1							0x00
#define SPL_PM_PRC_2							0x01
#define SPL_PM_PRC_4							0x02
#define SPL_PM_PRC_8							0x03
#define SPL_PM_PRC_16							0x04
#define SPL_PM_PRC_32							0x05
#define SPL_PM_PRC_64							0x06
#define SPL_PM_PRC_128							0x07


/* ********************* */
// TMP_CFG
// Temperature configuration register
// Addr: 0x07
// Default: 0x00

// TMP_EXT - External sensor
#define SPL_TMP_EXT_EN							0x01 << 7

// TMP_RATE - Temperature measurement rate
// For background mode
#define SPL_TMP_RATE_1							0x00 << 4
#define SPL_TMP_RATE_2							0x01 << 4
#define SPL_TMP_RATE_4							0x02 << 4
#define SPL_TMP_RATE_8							0x03 << 4
#define SPL_TMP_RATE_16							0x04 << 4
#define SPL_TMP_RATE_32							0x05 << 4
#define SPL_TMP_RATE_64							0x06 << 4
#define SPL_TMP_RATE_128						0x07 << 4

// TMP_PRC - Temperature oversampling rate
#define SPL_TMP_PRC_1							0x00
#define SPL_TMP_PRC_2							0x01
#define SPL_TMP_PRC_4							0x02
#define SPL_TMP_PRC_8							0x03
#define SPL_TMP_PRC_16							0x04
#define SPL_TMP_PRC_32							0x05
#define SPL_TMP_PRC_64							0x06
#define SPL_TMP_PRC_128							0x07


/* ********************* */
// MEAS_CFG
// Sensor operating mode and Status register
// Addr: 0x08
// Default: 0x00

// COEF_RDY - Coefficients ready
#define SPL_COEF_RDY							0x01 << 7

// SENSOR_RDY - Sensor ready
#define SPL_SENSOR_RDY							0x01 << 6

// TMP_RDY - Temperature measurement ready
#define SPL_TMP_RDY								0x01 << 5

// PRS_RDY - Pressure measurement ready
#define SPL_PRS_RDY								0x01 << 4

// MEAS_CTRL - Measurement control
// Standby
#define SPL_MEAS_CTRL_IDLE						0x00
// Command mode
#define SPL_MEAS_CTRL_PRS						0x01
#define SPL_MEAS_CTRL_TMP						0x02
// Background mode
#define SPL_MEAS_CTRL_CONT_PRS					0x05
#define SPL_MEAS_CTRL_CONT_TMP					0x06
#define SPL_MEAS_CTRL_CONT_PRS_TMP				0x07


/* ********************* */
// CFG_REG
// Interrupt and FIFO configuration register
// Addr: 0x09
// Default: 0x00

// INT_HL - INT on SDO pin level
#define SPL_INT_HIGH								0x01 << 7

// INT_FIFO - Interrupt when FIFO is full
#define SPL_INT_FIFO_EN								0x01 << 6

// INT_PRS - Interrupt on pressure measurement ready
#define SPL_INT_PRS_EN								0x01 << 5

// INT_TMP - Interrupt on temperature measurement ready
#define SPL_INT_TMP_EN								0x01 << 4

// T_SHIFT - Temperature result bit shift
// Must be used with temp oversampling > 8
#define SPL_T_SHIFT									0x01 << 3

// P_SHIFT - Pressure result bit shift
// Must be used with pres oversampling > 8
#define SPL_P_SHIFT									0x01 << 2

// FIFO_EN - Enable FIFO
#define SPL_FIFO_EN									0x01 << 1

// SPI_MODE - Select SPI mode
#define SPL_SPI_MODE_4W								0x00
#define SPL_SPI_MODE_3W								0x01


/* ********************* */
// INT_STS
// Interrupt Status register
// Addr: 0x0A
// Default: 0x00

// FIFO_FULL - FIFO full interrupt active
#define SPL_INT_FIFO_FULL							0x01 << 2

// INT_TMP - Temperature measurement rdy interrupt active
#define SPL_INT_TMP_STS								0x01 << 1

// INT_PRS - Pressure measurement rdy interrupt active
#define SPL_INT_PRS_STS								0x01


/* ********************* */
// FIFO_STS
// FIFO Status register
// Addr: 0x0B
// Default: 0x00

// FIFO_FULL_STS - FIFO full indicator
#define SPL_FIFO_FULL_STS							0x01 << 1

// FIFO_EMPTY_STS - FIFO empty indicator
#define SPL_FIFO_EMPTY_STS							0x01


/* ********************* */
// RESET
// Soft reset and FIFO flush register
// Addr: 0x0C
// Default: 0x00

// FIFO_FLSUH - FIFO Flush
// Clear ol data when measurement is read
#define SPL_FIFO_FLUSH								0x01 << 7

// SOFT_RST - Soft Reset
#define SPL_SOFT_RST								0x09


/* ********************* */
// ID
// Product ID and Revision ID
// Addr: 0x0D
// Default: 0x10

// PROD_ID - Product ID
#define SPL_PROD_ID									0x01 << 4

// REV_ID - Revision ID
#define SPL_REV_ID									0x00






/* -------
*
*
* Typedefs
*
*
------- */


typedef struct{
	// Coeffs
	int16_t coeffA[2];
	int32_t coeffB[2];
	int16_t coeffC[5];
	uint8_t coeff_raw[18];
	// Raw meas bytes
	uint8_t raw[6];
	// Scaling vars
	uint32_t kt;
	uint32_t kp;
	// Raw measurement assembled
	int32_t rawT;
	int32_t rawP;
	// Scaled measurements;
	float scT;
	float scP;
	// Temperature scaled
	float scT_acc;
	// Compensated measurements
	float compT;
	float compP;
	// Flags
	uint8_t triggered_P;
	uint8_t triggered_T;
	uint8_t reading;
	uint8_t rdy;
	uint16_t cntr;
	// Interface
	I2C_HandleTypeDef *i2c_Handle;
	// Measured data
	uint8_t meas_addr[6];
	// Altitude
	float altitude;
} SPL_data;




/* -------
*
*
* Functions
*
*
------- */


uint8_t SPL_Read_ID(SPL_data *input);
uint8_t SPL_Init(SPL_data *input, I2C_HandleTypeDef *hi2c, uint8_t mode, uint8_t temp_prc, uint8_t pres_prc);
uint8_t SPL_Read_Coeffs(SPL_data *input);
void SPL_Comp_P(SPL_data *input);
void SPL_Comp_T(SPL_data *input);
uint8_t SPL_SendAddr_DMA_P(SPL_data *input);
uint8_t SPL_SendAddr_DMA_T(SPL_data *input);
uint8_t SPL_ReadP_DMA(SPL_data *input);
uint8_t SPL_ReadT_DMA(SPL_data *input);
uint8_t SPL_ReadAll_DMA(SPL_data *input);
void SPL_ReadCplt_DMA(SPL_data *input);
uint8_t SPL_Trig_MeasP(SPL_data *input);
uint8_t SPL_Trig_MeasT(SPL_data *input);
void SPL_Raw_ConstructP(SPL_data *input);
void SPL_Raw_ConstructT(SPL_data *input);
uint8_t SPL_ReadPolling(SPL_data *input);
int32_t SPL_24to32_Convert(uint8_t *data);
int16_t SPL_12to16_Convert(uint8_t *data);
int32_t SPL_20to32_Convert(uint8_t *data);







#endif
