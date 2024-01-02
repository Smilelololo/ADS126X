/*
 * ADS126x.c
 *  Created: 		Dec 20, 2023
 *	Coded:			Smilelololo
 *  Updated: 		
 *	  
 *  ref: Michele Gazzarri, at https://github.com/dinamitemic/ADS126X/tree/master
 *	ref:
 */
#include <assert.h>
#include <string.h>
#include "main.h"
#include "ADS126x.h"	// All other required source files are declared here

//[todo] ADS126x_DATA_BUF_SIZE
#define  ADS126x_DATA_BUF_SIZE 6							// fixed to 6 bytes ( 1 byte status + 4 bytes data + 1 byte checksum)

/////////////////////////////////////////////////////////////////////////////////////
// hardware related
////////////////////////////////////////////////////////////////////////////////////
#if ADS126X_POWERDOWNMODE_SUPPORT == 1
	#define ADS126X_nPowerDown_nReset_Pin					ADS126X_nRST_Pin
	#define ADS126X_nPowerDown_nReset_GPIO_Port		ADS126X_nRST_GPIO_Port
#endif

#if ADS126X_CONVERSIONCONTROL_THROUGH_PIN == 1
	#define ADS126X_Start_Pin											ADS126X_START_Pin 
	#define ADS126X_Start_GPIO_Port								ADS126X_START_GPIO_Port
#endif

#if ADS126X_DATAREADY_THROUGH_SPI == 0
	#define ADS126X_nDataReady_Pin								ADS126X_nDRDY_Pin 				
	#define ADS126X_nDataReady_GPIO_Port					ADS126X_nDRDY_GPIO_Port
#endif

// SPI CS is assumed software controlled
#define ADS126X_nCS_Pin													SPI3_nCS_Pin
#define ADS126X_nCS_GPIO_Port										SPI3_nCS_GPIO_Port
#define ADS126X_SPI_Handle											hspi3									// the spi handle need to be accessible


/////////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////////
ADS126X_StructDef ads126x = { 0 };

/////////////////////////////////////////////////////////////////////////////////////
// local macros
////////////////////////////////////////////////////////////////////////////////////
#define ADS126x_CS_isDeactive()					(HAL_GPIO_ReadPin(ADS126X_nCS_GPIO_Port, ADS126X_nCS_Pin) == GPIO_PIN_SET)
#define ADS126x_CS_Active()							HAL_GPIO_WritePin(ADS126X_nCS_GPIO_Port, ADS126X_nCS_Pin, GPIO_PIN_RESET)
#define ADS126x_CS_Deactive()						HAL_GPIO_WritePin(ADS126X_nCS_GPIO_Port, ADS126X_nCS_Pin, GPIO_PIN_SET)

#define ADS126X_STATUSBYTE_ADC2_NEW_Msk			(1<<7)							//	0:	ADC2 data not new since the last ADC2 read operation;   1: ADC2 data new since the last ADC2 read operation;
#define ADS126X_STATUSBYTE_ADC1_NEW_Msk			(1<<6)							//	0:	ADC1 data not new since the last ADC2 read operation;   1: ADC1 data new since the last ADC2 read operation;
#define ADS126X_STATUSBYTE_EXTCLK_Msk				(1<<5)							//	0:	ADC clock is internal;			1: ADC clock is external
#define ADS126X_STATUSBYTE_REF_ALM_Msk			(1<<4)							//	0:	No alarm;										1:	Low reference alarm of ADC1, typically it is set if Vref <=0.4V.
#define ADS126X_STATUSBYTE_PGAL_ALM_Msk			(1<<3)							//	0:	No alarm;										1:	PGA low voltage alarm, the bit is set if the absolute voltage of either PGA output is less than Vavss+0.2V
#define ADS126X_STATUSBYTE_PGAH_ALM_Msk			(1<<2)							//	0:	No alarm;										1:	the bit is set if the absolate voltage of either PGA output is greater than Vavdd - 0.2V.
#define ADS126X_STATUSBYTE_PGAD_ALM_Msk			(1<<1)							//	0:	No alarm;										1:	the bit is set if the PGA differential output voltage exceeds +105% FS or -105% FS.
#define ADS126X_STATUSBYTE_RESET_Msk				(1<<0)							//	0:	no reset occurred since the RESET bit in power register last cleared by the user.		1:	Device reset occurred

// local function
void ADS126x_SendCommand(uint8_t command_byte){
	if (ADS126x_CS_isDeactive()) {
		ADS126x_CS_Active();
		while(ads126x.phspi->State == HAL_SPI_STATE_BUSY);
		ads126x.spi_txbytes[0] = command_byte;
		HAL_StatusTypeDef s = HAL_SPI_TransmitReceive_DMA( ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, 1);
		ADS126x_CS_Deactive();
	} else {
		while(ads126x.phspi->State == HAL_SPI_STATE_BUSY);
		ads126x.spi_txbytes[0] = command_byte;
		HAL_StatusTypeDef s = HAL_SPI_TransmitReceive_DMA( ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, 1);
	}	
}

void ADS126x_PowerUp(void){
#if ADS126X_POWERDOWNMODE_SUPPORT == 1
	if (HAL_GPIO_ReadPin(ADS126X_nPowerDown_nReset_GPIO_Port, ADS126X_nPowerDown_nReset_Pin) == GPIO_PIN_RESET) {
		HAL_GPIO_WritePin(ADS126X_nPowerDown_nReset_GPIO_Port, ADS126X_nPowerDown_nReset_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
	}
#else
	//not powerdown function
	assert(0);
#endif	
}

void ADS126x_PowerDown(void){
#if ADS126X_POWERDOWNMODE_SUPPORT == 1
	if (HAL_GPIO_ReadPin(ADS126X_nPowerDown_nReset_GPIO_Port, ADS126X_nPowerDown_nReset_Pin) == GPIO_PIN_SET) {
		HAL_GPIO_WritePin(ADS126X_nPowerDown_nReset_GPIO_Port, ADS126X_nPowerDown_nReset_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
	}
#else
	//not powerdown function
	assert(0);
#endif	
}

void ADS126x_Init(void) {

	ads126x.phspi = &ADS126X_SPI_Handle;
	
	ADS126x_Stop();
	
	ADS126x_PowerUp();

	ADS126x_ReadMultiRegister(0, ADS126X_NUM_REG, ads126x.RegData);
	
	if (ads126x.RegData[ADS126X_REG_POWER_Addr] &	ADS126X_REG_POWER_RESET_Msk) {
		ads126x.RegData[ADS126X_REG_POWER_Addr] &= 	~ADS126X_REG_POWER_RESET_Msk;						//set RESET from 1 to 0; reset event occurred if RESET == 1 again;
	}
	//ads126x.RegData[ADS126X_REG_POWER_Addr] |= 	ADS126X_REG_POWER_VBIAS_Msk;						//set VBIAS from 0(default) to 1 to enable the internal level shift voltage to the AIMCOM pin
	
	//ads126x.RegData[ADS126X_REG_POWER_Addr] &= 	~ADS126X_REG_POWER_INTREF_Msk;					//set INTREF from 1(default) to 0 to disable the internal voltage (2.5V) reference
	
	//ads126x.RegData[ADS126X_REG_INTERFACE_Addr] |= 	ADS126X_REG_INTERFACE_TIMEOUT_Msk;	//set TIMEOUT from 0 (default) to 1 to enable the interface automatic time-out
	
	//ads126x.RegData[ADS126X_REG_INTERFACE_Addr] &= 	~ADS126X_REG_INTERFACE_STATUS_Msk;	//set STATUS from 1 (default) to 0 to disable the interface status byte output
	
	//ads126x.RegData[ADS126X_REG_INTERFACE_Addr] &= 	~ADS126X_REG_INTERFACE_CRC_Msk;			//clear CRC from 0b10 (checksun mode) to 0b00 to disable the checksun output
	//ads126x.RegData[ADS126X_REG_INTERFACE_Addr] |= 	ADS126X_REG_INTERFACE_CRC_Value_CRC;//change CRC from 0b00 (disabled) to CRC mode
	
	//ads126x.RegData[ADS126X_REG_MODE0_Addr] |= 	ADS126X_REG_MODE0_REFREV_Msk;						//change reference polarity from 0 (normal polarity) to reverse polarity
	
	//ads126x.RegData[ADS126X_REG_MODE0_Addr] |= 	ADS126X_REG_MODE0_RUNMODE_Msk;					//select ADC conversion rum mode from 0 (continuous coversion, default) to pulse conversion (one shot conversion)
	
	//ads126x.RegData[ADS126X_REG_MODE0_Addr] |= 	ADS126X_REG_MODE0_CHOPMODE_Value_INPUTCHOP_IDACROTATION;					//select chop mode from 0b00 (input chop and IDAC rotation disabled, default) to input chop and IDAC rotation enabled
	//ads126x.RegData[ADS126X_REG_MODE0_Addr] |= 	ADS126X_REG_MODE0_CHOPMODE_Value_INPUTCHOP;												//select chop mode from 0b00 (input chop and IDAC rotation disabled, default) to input chop enabled
	//ads126x.RegData[ADS126X_REG_MODE0_Addr] |= 	ADS126X_REG_MODE0_CHOPMODE_Value_IDACROTATION;										//select chop mode from 0b00 (input chop and IDAC rotation disabled, default) to IDAC rotation enabled

	//ads126x.RegData[ADS126X_REG_MODE0_Addr] |= ADS126X_REG_MODE0_CONVERSIONDELAY_Value_8us7;											// additional delay from conversion start to the beginning of the actual conversion
	
	//ads126x.RegData[ADS126X_REG_MODE1_Addr] &= ~ADS126X_REG_MODE1_FILTER_Msk;						// digital filter from FIR mode (0b100, default) to Sinc1 mode (0b000);
	//ads126x.RegData[ADS126X_REG_MODE1_Addr] |= ADS126X_REG_MODE1_FILTER_Value_SINC4;		// digital filter from FIR mode (0b000, previous) to Sinc4 mode (0b011);

	//ads126x.RegData[ADS126X_REG_MODE1_Addr] |= ADS126X_REG_MODE1_SENSORBIASADC_Value_ADC2;	// sensor bias is connected from ADC1(0, default) to ADC2 (1);
	
	//ads126x.RegData[ADS126X_REG_MODE1_Addr] |= ADS126X_REG_MODE1_SENSORBIASPOL_Value_PULLDOWN;	// sensor bias polarity is changed from pull-up mode (0, default) to pull-down mode(1);
	
	//ads126x.RegData[ADS126X_REG_MODE1_Addr] |= ADS126X_REG_MODE1_SENSORBIASMAG_Value_10uA;			// sensor bias magnitude is changed from no current or resistor to (0b000, default) to 10uA(0b011);
	
	//ads126x.RegData[ADS126X_REG_MODE2_Addr] |= ADS126X_REG_MODE2_BYPASS_Msk;										// PGA enabled (0, default) to PGA bypassed (1);
	
	//ads126x.RegData[ADS126X_REG_MODE2_Addr] |= ADS126X_REG_MODE1_GAIN_Value_32X;								// PGA gain from 1V/V (0b000, default) to 32V/V (0b101);
	
	//ads126x.RegData[ADS126X_REG_MODE2_Addr] &= ~ADS126X_REG_MODE2_DATARATE_Msk;									// Data Rate from 20SPS (0b0100, default) to 2.5SPS (0b0000);
	//ads126x.RegData[ADS126X_REG_MODE2_Addr] |= ADS126X_REG_MODE1_DATARATE_Value_4800SPS;				// Data Rate from (0b0000) to 4800SPS (0b1011);

	//ads126x.RegData[ADS126X_REG_INPUTMUX_Addr] |= ADS126X_REG_INPUTMUX_MUXP_Value_AIN2;					// Positive Input Multiplexer selects from AIN0 (0b0000, default) to AIN2 (0b0010)
	
	//ads126x.RegData[ADS126X_REG_INPUTMUX_Addr] &= ~ADS126X_REG_INPUTMUX_MUXN_Msk;								// Negetive Input Multiplexer selects from AIN1 (0b0001, default) to AIN0 (0b0000)
	//ads126x.RegData[ADS126X_REG_INPUTMUX_Addr] |= ADS126X_REG_INPUTMUX_MUXN_Value_AIN3;					// Negetive Input Multiplexer selects from AIN0 (0b0000) to AIN3 (0b0011)
	
	//[todo] calibration 

	//[todo] IDACMUX/IDACMAG
	
	//ads126x.RegData[ADS126X_REG_REFMUX_Addr] |= ADS126X_REG_REFMUX_P_Value_AIN4;								// ADC voltage Reference positive from internal 2.5V referenc positive(0b000, default) to AIN4 (0b011)

	//ads126x.RegData[ADS126X_REG_REFMUX_Addr] |= ADS126X_REG_REFMUX_N_Value_AIN5;								// ADC voltage Reference negative from internal 2.5V referenc negative(0b000, default) to AIN5 (0b011)

	//[todo] TDACP/TDACN
	
	//[todo] GPIO

	if ( (ads126x.RegData[ADS126X_REG_ID_Addr] & ADS126X_REG_ID_DEVICEID_Msk ) == ADS126X_REG_ID_DEVICEID_Value_ADS1263 )
	{
		//[todo] ADC2
	}

	//ADS126x_WriteMultiRegister(0, ADS126X_NUM_REG, ads126x.RegData);
	// continous mode does not work when bytes > 25 are updated together.
	ADS126x_WriteMultiRegister(0, 25, ads126x.RegData);	
	
}

void ADS126x_ReadMultiRegister(uint32_t StartAddr, uint32_t NumRegs, uint8_t *pdata){
	// interface check
	assert( (~0x1F&StartAddr) == 0);
	assert( NumRegs <= ADS126X_NUM_REG);
	assert( pdata );
	
	ads126x.spi_txbytes[0] = 0x20 | StartAddr;
	ads126x.spi_txbytes[1] = NumRegs - 1;
	if (ADS126x_CS_isDeactive()) {
		ADS126x_CS_Active();
		while(ads126x.phspi->State != HAL_SPI_STATE_READY);
		if (HAL_OK != HAL_SPI_TransmitReceive( ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, NumRegs + 2, 1000)) {
			Error_Handler();
		}
		ADS126x_CS_Deactive();
	} else {
		while(ads126x.phspi->State != HAL_SPI_STATE_READY);
		if (HAL_OK != HAL_SPI_TransmitReceive( ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, NumRegs + 2, 1000)){
			Error_Handler();
		}
	}	
	memcpy(pdata, ads126x.spi_rxbytes+2, NumRegs);
}

void ADS126x_WriteMultiRegister(uint32_t StartAddr, uint32_t NumRegs, uint8_t *pdata){
	// interface check
	assert( (~0x1F&StartAddr) == 0);
	assert( NumRegs <= ADS126X_NUM_REG);
	assert( pdata );
	
	ads126x.spi_txbytes[0] = 0x40 | StartAddr;
	ads126x.spi_txbytes[1] = NumRegs - 1;
	memcpy(ads126x.spi_txbytes+2, pdata, NumRegs);
	if (ADS126x_CS_isDeactive()) {
		ADS126x_CS_Active();
		while(ads126x.phspi->State != HAL_SPI_STATE_READY);
		if (HAL_OK != HAL_SPI_TransmitReceive( ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, NumRegs + 2, 1000)){
			Error_Handler();
		}
		ADS126x_CS_Deactive();
	} else {
		while(ads126x.phspi->State != HAL_SPI_STATE_READY);
		if (HAL_OK != HAL_SPI_TransmitReceive( ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, NumRegs + 2, 1000)){
			Error_Handler();
		}
	}	
}

void ADS126x_Start(void){
#ifdef	ADS126X_CONVERSIONCONTROL_THROUGH_PIN
	HAL_GPIO_WritePin(ADS126X_Start_GPIO_Port, ADS126X_Start_Pin, GPIO_PIN_SET);
#else
	ADS126x_SendCommand(ADS126X_CMD_START1);
#endif
	ads126x.is_ADC_running = 1;
}

void ADS126x_Stop(void){
#ifdef	ADS126X_CONVERSIONCONTROL_THROUGH_PIN
	HAL_GPIO_WritePin(ADS126X_Start_GPIO_Port, ADS126X_Start_Pin, GPIO_PIN_RESET);
#else
	ADS126x_SendCommand(ADS126X_CMD_STOP1);
#endif
	ads126x.is_ADC_running = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if((GPIO_Pin == ADS126X_nDataReady_Pin ) && ads126x.is_ADC_running) {
		if (ads126x.is_ADC_dataReady == 1) {
			ads126x.missing_ADC_dataReady_count++;
		}
		else {
			ads126x.is_ADC_dataReady = 1;
			ADS126x_CS_Active();
			HAL_SPI_TransmitReceive_DMA(ads126x.phspi, ads126x.spi_txbytes, ads126x.spi_rxbytes, ADS126x_DATA_BUF_SIZE);
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if ((hspi -> Instance == SPI3) && ads126x.is_ADC_running){
		// deactive CS
		ADS126x_CS_Deactive();
		// notice spi has new data to process
		if ( ads126x.is_SPI_dataReady == 1){
			ads126x.missing_SPI_dataReady_count++;
		}
		else {
			ads126x.is_SPI_dataReady = 1;
		}
		ads126x.is_ADC_dataReady = 0;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	//[todo]
	//hspi->ErrorCode
}

//	invoke ADS126x_PushDatatoCircularBuf() when there is new data ready through spi from ADC as follows
//
//	if (ads126x.is_SPI_dataReady == 1)
//	{
//		ADS126x_PushDatatoCircularBuf();
//		ads126x.is_SPI_dataReady = 0;
//	}
void ADS126x_PushDatatoCircularBuf(void){
	if(ads126x.spi_rxbytes[0] == (ADS126X_STATUSBYTE_ADC1_NEW_Msk | ADS126X_STATUSBYTE_EXTCLK_Msk))
	{
		if ( ((ads126x.spi_rxbytes[1]+ads126x.spi_rxbytes[2]+ads126x.spi_rxbytes[3]+ads126x.spi_rxbytes[4] + 0x9B)& 0xFF) == ads126x.spi_rxbytes[5] )
		{
			uint32_t next_write_index = ads126x.data_circularBuf.write_index + 1;
			if ( next_write_index == ADS126X_DATA_CIRCULARBUF_SIZE) {
				next_write_index = 0;
			}
			if (ads126x.data_circularBuf.read_index == next_write_index) {
				ads126x.data_circularBuf.overrun_count++;
			}
			else {
				ads126x.data_circularBuf.buffer[ads126x.data_circularBuf.write_index] = 
					(ads126x.spi_rxbytes[1] << 24) |		//Data MSB
					(ads126x.spi_rxbytes[2] << 16) |
					(ads126x.spi_rxbytes[3] <<  8) |
					(ads126x.spi_rxbytes[4] <<  0);			//Data LSB
				ads126x.data_circularBuf.write_index = next_write_index;
			}
		}
		else {
			ads126x.warning_ADC_checksum_count++;
		}
	}
	else {
		ads126x.warning_ADC_status_count++;
	}
}

// parameters: 
//		data_buf, array with size >= the value of *length during input.
//		*length:  input with the value which is the max number of data expected to get, output with the number of data actually to get.
void ADS126x_GetDatafromCircularBuf(uint32_t* data_buf, uint32_t *length){
	uint32_t data_number;
	if (ads126x.data_circularBuf.read_index != ads126x.data_circularBuf.write_index ) {			// buffer is not empty
		
		if (ads126x.data_circularBuf.read_index < ads126x.data_circularBuf.write_index ) {
			data_number = ads126x.data_circularBuf.write_index - ads126x.data_circularBuf.read_index;
			if (data_number < *length ) {
				*length = data_number;
			}
			memcpy(data_buf, ads126x.data_circularBuf.buffer+ads126x.data_circularBuf.read_index, 4**length);
			ads126x.data_circularBuf.read_index += *length;
		}
		else {
			data_number = ADS126X_DATA_CIRCULARBUF_SIZE - ads126x.data_circularBuf.read_index;
			if (*length > data_number ) {
				*length = data_number;
			}
			memcpy(data_buf, ads126x.data_circularBuf.buffer+ads126x.data_circularBuf.read_index, 4**length);
			ads126x.data_circularBuf.read_index += *length;
			if (ads126x.data_circularBuf.read_index == ADS126X_DATA_CIRCULARBUF_SIZE){
				ads126x.data_circularBuf.read_index = 0;
			}
		}
	}
}