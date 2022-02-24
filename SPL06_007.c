#include "SPL06_007.h"
#include "main.h"

/* --------

	Functions

-------- */


uint8_t SPL_Read_ID(SPL_data *input){
	
	/* --------

	Read the device ID

	-------- */

	HAL_StatusTypeDef res;
	uint8_t buf[3];

	buf[0] = SPL_ID;				//	ID reg location
	buf[1] = SPL_ADDR_W;			//	Write mode
	buf[2] = SPL_ADDR_R;			//	Read mode

	// write the address of the registers to be read
	res = HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)buf[1], &buf[0], 1, HAL_MAX_DELAY); 
	
	if ( res != HAL_OK ){
		return 2;
	}
	else{
		// read the value of regs
		res = HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)buf[2], &buf[0], 1, HAL_MAX_DELAY);

		if ( res != HAL_OK ){
			return 3;
		}
	}
	
	// check whether the id was received correctly (reset value of reg)
	if ( buf[0] != (SPL_PROD_ID |  SPL_REV_ID) ){	
		return 1;
	}
	else{
		return 0;
	}
}


uint8_t SPL_Init(SPL_data *input, I2C_HandleTypeDef *hi2c, uint8_t mode, uint8_t temp_prc, uint8_t pres_prc){
	
	/* --------

	Initialize the device

	-------- */
	


	// Set interface
	input->i2c_Handle = hi2c;
	
	// Set regs to be written
	uint8_t buf[6];
	
	// Reset sensor
	// Fill up buffer
	buf[0] = SPL_RESET;
	buf[1] = SPL_SOFT_RST;

	// Send reset command
	if( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, buf, 2, HAL_MAX_DELAY) != HAL_OK ){
		return 1;
	}

	// Fill up buffer
	buf[0] = SPL_MEAS_CFG;
	buf[1] = mode;
	buf[2] = SPL_PRS_CFG;
	buf[3] = pres_prc;
	buf[4] = SPL_TMP_CFG;
	buf[5] = temp_prc;
	
	
	// Waste 100 ms to secure sensor restart
	HAL_Delay(100);
	
	// Set configuration
	for ( uint8_t i = 0; i < 3; i++ ){
		
		// Write config addr and setting
		if ( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &buf[2*i], 2, HAL_MAX_DELAY) != HAL_OK ){
			return 1;
		}
	}
	
	// Set scaling parameters (TEMP)
	if ( temp_prc == SPL_TMP_PRC_1 ){
		input->kt = 524288;
	}
	else if ( temp_prc == SPL_TMP_PRC_2 ){
		input->kt = 1572864;
	}
	else if ( temp_prc == SPL_TMP_PRC_4 ){
		input->kt = 3670016;
	}
	else if ( temp_prc == SPL_TMP_PRC_8 ){
		input->kt = 7864320;
	}
	else if ( temp_prc == SPL_TMP_PRC_16 ){
		input->kt = 253952;
	}
	else if ( temp_prc == SPL_TMP_PRC_32 ){
		input->kt = 516096;
	}
	else if ( temp_prc == SPL_TMP_PRC_64 ){
		input->kt = 1040384;
	}
	else if ( temp_prc == SPL_TMP_PRC_128 ){
		input->kt = 2088960;
	}
	else{
		return 4;
	}
	
	
	// Set scaling parameters (PRES)
	if ( pres_prc == SPL_PM_PRC_1 ){
		input->kp = 524288;
	}
	else if ( pres_prc == SPL_PM_PRC_2 ){
		input->kp = 1572864;
	}
	else if ( pres_prc == SPL_PM_PRC_4 ){
		input->kp = 3670016;
	}
	else if ( pres_prc == SPL_PM_PRC_8 ){
		input->kp = 7864320;
	}
	else if ( pres_prc == SPL_PM_PRC_16 ){
		input->kp = 253952;
	}
	else if ( pres_prc == SPL_PM_PRC_32 ){
		input->kp = 516096;
	}
	else if ( pres_prc == SPL_PM_PRC_64 ){
		input->kp = 1040384;
	}
	else if ( pres_prc == SPL_PM_PRC_128 ){
		input->kp = 2088960;
	}
	else{
		return 5;
	}
	
	
	// Set measured data locations
	input->meas_addr[0] = SPL_PSR_B2;
	input->meas_addr[1] = SPL_TMP_B2;
	input->meas_addr[2] = SPL_MEAS_CFG;
	input->meas_addr[3] = SPL_MEAS_CTRL_PRS;
	input->meas_addr[4] = SPL_MEAS_CFG;
	input->meas_addr[5] = SPL_MEAS_CTRL_TMP;
	
	
	// Set buffer for cfg reg
	buf[0] = SPL_CFG_REG;
	
	// Set bit shift setting
	if ( (temp_prc << 4) > SPL_PM_RATE_8 && pres_prc > SPL_PM_PRC_8 ){
		// Oversampling is greater for both temp and pres
		buf[1] = SPL_T_SHIFT | SPL_P_SHIFT;
		
		// Write setting
		if ( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, buf, 2, HAL_MAX_DELAY) != HAL_OK ){
			return 6;
		}
	}
	else if ( (temp_prc << 4) > SPL_PM_RATE_8 && pres_prc <= SPL_PM_PRC_8 ){
		// Oversampling is greater for temp
		buf[1] = SPL_T_SHIFT;
		
		// Write setting
		if ( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, buf, 2, HAL_MAX_DELAY) != HAL_OK ){
			return 6;
		}
	}
	else if ( (temp_prc << 4) <= SPL_PM_RATE_8 && pres_prc > SPL_PM_PRC_8 ){
		// Oversampling is greater for pres
		buf[1] = SPL_P_SHIFT;
		
		// Write setting
		if ( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, buf, 2, HAL_MAX_DELAY) != HAL_OK ){
			return 6;
		}
	}


	// Check configuration
	// Write addr
	buf[0] = SPL_PRS_CFG;

	if( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, buf, 1, HAL_MAX_DELAY) != HAL_OK ){
		return 1;
	}

	if( HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)SPL_ADDR_R, buf, 2, HAL_MAX_DELAY) != HAL_OK ){
		return 2;
	}

	if( (buf[0] != pres_prc) & 0b11 || (buf[1] != temp_prc) & 0b11 ){
		return 7;
	}
	
	
	// Read coeffs
	if ( SPL_Read_Coeffs(input) != 0 ){
		return 3;
	}
	
	// write the address of the registers to be read
	// if ( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &buf[0], 1, HAL_MAX_DELAY) != HAL_OK ){
	// 	return 1;
	// }
	
	return SPL_Read_ID(input);
}


uint8_t SPL_Read_Coeffs(SPL_data *input){
	
	/* --------

	Read out the coefficients
	
	-------- */
	
	// Set regs to be written
	uint8_t buf[18];
	
	// Write in buf
	buf[0] = SPL_COEF;
	
	// Write coeff addr
	if ( HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, buf, 1, HAL_MAX_DELAY) != HAL_OK ){
		return 1;
	}
	
	// Read coeffs
	if (HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)SPL_ADDR_R, buf, SPL_COEF_LEN, HAL_MAX_DELAY) != HAL_OK ){
		return 2;
	}
	
	// Put raw data in struct
	for ( uint8_t i = 0; i < SPL_COEF_LEN; i++ ){
		input->coeff_raw[i] = buf[i];
	}
	
	// Buffer for converting vars
	uint8_t buf2[3];

	// Assemble coeffs (A0)
	buf2[0] = input->coeff_raw[0];
	buf2[1] = input->coeff_raw[1] & 0xF0;
	// input->coeffA[0] = (input->coeff_raw[0] << 4) + (input->coeff_raw[1] >> 4);
	input->coeffA[0] = SPL_12to16_Convert(buf2);										// 12-bit "c0"

	// Assemble coeffs (A1)
	buf2[0] = ( input->coeff_raw[1] & 0x0F << 4 ) | ( input->coeff_raw[2] & 0xF0 >> 4 );
	buf2[1] = input->coeff_raw[2] & 0x0F << 4;
	// input->coeffA[1] = (( input->coeff_raw[1] & 0x0F ) << 8) + input->coeff_raw[2];
	input->coeffA[1] = SPL_12to16_Convert(buf2);										// 12-bit "c1"

	// Assemble coeffs (B0)
	buf2[0] = input->coeff_raw[3];
	buf2[1] = input->coeff_raw[4];
	buf2[2] = input->coeff_raw[5] & 0xF0;
	// input->coeffB[0] = (input->coeff_raw[3] << 12) + (input->coeff_raw[4] << 4) + (input->coeff_raw[5] >> 4);
	input->coeffB[0] = SPL_20to32_Convert(buf2);										// 20-bit "c00"

	// Assemble coeffs (B1)
	buf2[0] = ( (input->coeff_raw[5] & 0x0F ) << 4 ) | input->coeff_raw[6] >> 4;
	buf2[1] = ( ( input->coeff_raw[6] & 0x0F ) << 4 ) | input->coeff_raw[7] >> 4;
	buf2[2] = ( input->coeff_raw[7] & 0x0F ) << 4;
	// input->coeffB[1] = input->coeff_raw[7] + (input->coeff_raw[6] << 8) + ((input->coeff_raw[5] & 0x0F) << 16);	// 20-bit "c10"
	input->coeffB[1] = SPL_20to32_Convert(buf2);
	
	// Assemble coeffs (C)
	input->coeffC[0] = (input->coeff_raw[8] << 8) + input->coeff_raw[9];				// 16-bit "c01"
	input->coeffC[1] = (input->coeff_raw[10] << 8) + input->coeff_raw[11];				// 16-bit "c11"
	input->coeffC[2] = (input->coeff_raw[12] << 8) + input->coeff_raw[13];				// 16-bit "c20"
	input->coeffC[3] = (input->coeff_raw[14] << 8) + input->coeff_raw[15];				// 16-bit "c21"
	input->coeffC[4] = (input->coeff_raw[16] << 8) + input->coeff_raw[17];				// 16-bit "c30"
	
	// Return from fcn
	return 0;
}

/*
 * Compensate measurements
 */

void SPL_Comp_P(SPL_data *input){
	
	/* --------

	Compensate pressure
	
	-------- */
	
	// Calculate scaled value
	input->scP = (float)(input->rawP) / (float)(input->kp);
	
	// Compensate pres
	input->compP = input->coeffB[0] + input->scP * ( input->coeffB[1] + input->scP * (input->coeffC[2] + input->scP * input->coeffC[4] ) );
	input->compP += input->scT_acc + input->scT * input->scP * ( input->coeffC[1] + input->scP * input->coeffC[3] );
}


void SPL_Comp_T(SPL_data *input){
	
	/* --------

	Compensate temperature
	
	-------- */
	
	// Calculate scaled value
	input->scT = (float)(input->rawT) / (float)(input->kt);
	
	// Compensate temp
	input->compT = (float)(input->coeffA[0]) / 2 + input->coeffA[1] * input->scT;

	// Accelerate pres calc term
	input->scT_acc = input->scT * input->coeffC[0];
}

/*
 * Polling read
 */


// Read data in polling mode
uint8_t SPL_ReadPolling(SPL_data *input){
	/*
	 *
	 * Read in polling mode
	 * Read temp and pres as well
	 *
	 */

	/*
	 * Measure T
	 */

	// Trigger T
	if(HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[4], 2, HAL_MAX_DELAY) != HAL_OK){
		return 1;
	}

	// Delay for measurement to finish
	HAL_Delay(20);

	// Send addr for reading
	if(HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[1], 1, HAL_MAX_DELAY) != HAL_OK){
		return 1;
	}

	// Read measurement
	if(HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)SPL_ADDR_R, &input->raw[3], 3, HAL_MAX_DELAY) != HAL_OK){
		return 2;
	}

	// Construct raw
	SPL_Raw_ConstructT(input);

	// Float calc
	SPL_Comp_T(input);

	/*
	 * Measure P
	 */

	// Trigger P
	if(HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[2], 2, HAL_MAX_DELAY) != HAL_OK){
		return 1;
	}

	// Delay for measurement to finish
	HAL_Delay(20);

	// Send addr for reading
	if(HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[0], 1, HAL_MAX_DELAY) != HAL_OK){
		return 1;
	}

	// Read measurement
	if(HAL_I2C_Master_Receive(input->i2c_Handle, (uint16_t)SPL_ADDR_R, &input->raw[0], 3, HAL_MAX_DELAY) != HAL_OK){
		return 2;
	}

	// Construct raw
	SPL_Raw_ConstructP(input);

	// Float calc
	SPL_Comp_P(input);

	/*
	 * Trigger new meas
	 */

	// Trigger P
	if(HAL_I2C_Master_Transmit(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[2], 2, HAL_MAX_DELAY) != HAL_OK){
		return 1;
	}
	input->triggered_P = 1;

	// Return on success
	return 0;
}

/*
 * DMA read
 */


// SEND i2c dma addr
uint8_t SPL_SendAddr_DMA_P(SPL_data *input){
	/*
	*
	*	Read SPL data DMA P
	*	Starting DMA by sending start ADDR
	*
	*/

	if(HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[0], 1) == HAL_OK){
		input->rdy = 0;
		input->reading = 1;
		return 0;
	}
	else{
		return 1;
	}
}

uint8_t SPL_SendAddr_DMA_T(SPL_data *input){
	/*
	*
	*	Read SPL data DMA P
	*	Starting DMA by sending start ADDR
	*
	*/

	if(HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[1], 1) == HAL_OK){
		input->rdy = 0;
		input->reading = 1;
		return 0;
	}
	else{
		return 1;
	}
}


// READ i2c dma msg
uint8_t SPL_ReadP_DMA(SPL_data *input){
	/*
	*
	*	Read SPL data DMA
	*	Read pres or temp using DMA
	*	Put data in SPL struct
	*
	*	Put this function into:
	*	HAL_I2C_MasterTxCpltCallback();
	*
	*/

	if( HAL_I2C_Master_Receive_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_R, &input->raw[0], 3) == HAL_OK ){
		input->triggered_P = 0;
		return 0;
	}
	else{
		return 1;
	}
}

// READ i2c dma msg
uint8_t SPL_ReadT_DMA(SPL_data *input){
	/*
	*
	*	Read SPL data DMA
	*	Read pres or temp using DMA
	*	Put data in SPL struct
	*
	*	Put this function into:
	*	HAL_I2C_MasterTxCpltCallback();
	*
	*/

	if( HAL_I2C_Master_Receive_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_R, &input->raw[3], 3) == HAL_OK ){
		input->triggered_T = 0;
		return 0;
	}
	else{
		return 1;
	}
}


// READ i2c dma msg
uint8_t SPL_ReadAll_DMA(SPL_data *input){
	/*
	*
	*	Read SPL data DMA
	*	Read all data using DMA
	*	Put data in SPL struct
	*
	*	Put this function into:
	*	HAL_I2C_MasterTxCpltCallback();
	*
	*/

	if( HAL_I2C_Master_Receive_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_R, &input->raw[0], 6) == HAL_OK ){
		return 0;
	}
	else{
		return 1;
	}
}


// Read dma complete
void SPL_ReadCplt_DMA(SPL_data *input){
	/*
	*
	*	Read SPL data DMA
	*	Clear flag when data is read
	*	Save raw data
	*
	*	Put this function into:
	*	HAL_I2C_MasterRxCpltCallback();
	*
	*/

	// Reset I2C flag
	input->reading = 0;

	// Set I2C flag
	input->rdy = 1;
}


// Trigger measurement P
uint8_t SPL_Trig_MeasP(SPL_data *input){
	/*
	*
	*	Trigger SPL measurement
	*	Set flag when triggered
	*
	*/

	if(HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[2], 2) == HAL_OK){
		input->triggered_P = 1;
		return 0;
	}
	else{
		return 1;
	}
}


// Trigger measurement T
uint8_t SPL_Trig_MeasT(SPL_data *input){
	/*
	*
	*	Trigger SPL measurement
	*	Set flag when triggered
	*
	*/

	if(HAL_I2C_Master_Transmit_DMA(input->i2c_Handle, (uint16_t)SPL_ADDR_W, &input->meas_addr[4], 2) == HAL_OK){
		input->triggered_T = 1;
		return 0;
	}
	else{
		return 1;
	}
}


/*
 * Construct raw data
 */


// Construct raw data P
void SPL_Raw_ConstructP(SPL_data *input){
	/*
	*
	*	When raw data is read put bytes together
	*	PRES
	*
	*/
	
	// input->rawP = (int32_t)((input->raw[0] << 16) + (input->raw[1] << 8) + input->raw[2]);

	input->rawP = SPL_24to32_Convert(&input->raw[0]);
}


// Construct raw data T
void SPL_Raw_ConstructT(SPL_data *input){
	/*
	*
	*	When raw data is read put bytes together
	*	TEMP
	*
	*/
	
	// input->rawT = (int32_t)((input->raw[3] << 16) + (input->raw[4] << 8) + input->raw[5]);

	input->rawT = SPL_24to32_Convert(&input->raw[3]);
}


// Convert 24bit 2s complementer to int32_t
int32_t SPL_24to32_Convert(uint8_t *data){
	/*
	*
	*	Convert 24bit to 32 bit data
	*
	*/

	return ( (*data << 24) | (*(data + 1) << 16) | (*(data + 2) << 8) ) >> 8;
}


// Convert 24bit 2s complementer to int32_t
// CORRECT
int16_t SPL_12to16_Convert(uint8_t *data){
	/*
	*
	*	Convert 24bit to 32 bit data
	*
	*/

	return ( (*data << 8) | ( *(data + 1) ) ) >> 4;
}


// Convert 24bit 2s complementer to int32_t
// CORRECT
int32_t SPL_20to32_Convert(uint8_t *data){
	/*
	*
	*	Convert 24bit to 32 bit data
	*
	*/

	return ( (*data << 24) | (*(data + 1) << 16) | (*(data + 2) << 8) ) >> 12;
}
