/*
 * ADS126x.h
 *  Created: 		Dec 20, 2023
 *	Coded:			Smilelololo
 *  Updated: 		
 *	  
 *  ref: Michele Gazzarri, at https://github.com/dinamitemic/ADS126X/tree/master
 *	ref:
 */
// [todo] only support the data output default format (status byte + 4 bytes of data + checksum byte)
// [todo] data ready remapped on SPI MISO
// [todo] functions related to ADS1263 ADC2;
// [todo] functions related to ADS126X GPIO;

#ifndef ADS126X_H_
#define ADS126X_H_

#include <stdint.h>
#include "stm32f7xx_hal.h"

/////////////////////////////////////////////////////////////////////////////////////
// hardware interface
////////////////////////////////////////////////////////////////////////////////////
#define ADS1263																				//	Additional definitions to support ADS1263 additional features
#define ADS126X_POWERDOWNMODE_SUPPORT						1			// 	0: power down mode is not support, reset through command; 1: power down mode is supported (through nRESET/nPOWERDOWN pin), also reset through this pin too
#define ADS126X_DATAREADY_THROUGH_SPI						0			// 	0: data ready is indicated through a dedicated pin nDRDY; 1: data ready is multiplexed on the spi MISO/DOUT/nDRDY
#define ADS126X_CONVERSIONCONTROL_THROUGH_PIN		1			//	0: start/stop adc conversion through command;							1: start/stop adc conversion through the dedicated pin
/////////////////////////////////////////////////////////////////////////////////////
// ADS126X Command
////////////////////////////////////////////////////////////////////////////////////
	/* SPI Commands */
	#define ADS126X_CMD_NOP													(0x00)			/* No operation */
	#define ADS126X_CMD_RESET 											(0x06)			/* Reset the ADC, LSB does ont matter*/
	#define ADS126X_CMD_START1											(0x08)			/* Start ADC1 conversions, LSB does ont matter */
	#define ADS126X_CMD_STOP1												(0x0B)			/* Stop ADC1 conversions, LSB does ont matter */

	#define ADS126X_CMD_RDATA1											(0x12)			/* Read ADC1 data, LSB does ont matter */

	#define ADS126X_CMD_SYOCAL1											(0x16)			/* ADC1 system offset calibration */
	#define ADS126X_CMD_SYGCAL1											(0x17)			/* ADC system gain calibration */
	#define ADS126X_CMD_SFOCAL1											(0x19)			/* ADC1 self offset calibration */
	
	//Multi-Byte Commands, Register data read and write
	//note that combining the following with register address to form the first opcode byte.
	
	
	// The RREG opcode consists of two bytes. see datasheet page 86
	// The first byte specifies the starting register address: 001r rrrr : where r rrrr is the starting register address.
	// The second opcode byte is the number of registers to read: 000n nnnn : where n nnnn is the number of registers to read minus 1.
	// if the byte count exceeds the last register address, the ADC begins to output zero data, the address pointer does not wrap.
	// During the register read operation, if ADC1 data are ready, the conversion data are only loaded to the register to be read by RDATA1 later, not the output shift register to avoid data contention.
	#define ADS126X_CMD_RREG												(0x20)			/* Read registers */
	
	// The WREG opcode consists of two bytes.  see datasheet page 87
	// The first byte specifies the starting register address: 010r rrrr, where r rrrr is the starting register address
	// The second opcode byte is the number of registers to write (minus one)
	// if hte byte count exceeds the last register address, the ADC ignores the data, the address pointer does not wrap.
	// Writing new data to certain registers results in a reset of ADC1 or ADC2 conversions, as specified in the ADC restart column in the Register Maps.
	#define ADS126X_CMD_WREG												(0x40)			/* Write registers */

/* Additional ADS1263 Commands */
#ifdef ADS1263

	#define ADS126X_CMD_START2											(0x0C)			/* Start ADC2 conversions, LSB does ont matter */
	#define ADS126X_CMD_STOP2												(0x0E)			/* Stop ADC2 conversions, LSB does ont matter */
	
	#define ADS126X_CMD_RDATA2											(0x14)			/* Read ADC2 data, LSB does ont matter */
	
	#define ADS126X_CMD_SYOCAL2											(0x1B)			/* ADC2 system offset calibration */
	#define ADS126X_CMD_SYGCAL2											(0x1C)			/* ADC2 system gain calibration */
	#define ADS126X_CMD_SFOCAL2											(0x1E)			/* ADC2 self offset calibration */
	
#endif

/////////////////////////////////////////////////////////////////////////////////////
// ADS126X Register
////////////////////////////////////////////////////////////////////////////////////
// Typically, register changes take effect immediately after the data are written. 
// However, if the registers are part of a group, then the data are written only after all data for the grouped registers in the write block has been sent.

/* Register Addresses */

	/* 			register										address					default		A->D restart  Group Update	Bit 7					6					5					4					3					2					1					0					*/			
	
	#define ADS126X_REG_ID_Addr					(0x00)			/* 	xxh																	| Dev_ID[2:0], 000:1262; 001:1263	| Rev_ID[4:0]																			|	*/
	
			#define ADS126X_REG_ID_DEVICEID_Pos												(5U)
			#define ADS126X_REG_ID_DEVICEID_Bits											(3U)
			#define ADS126X_REG_ID_DEVICEID_Msk												(((1U << ADS126X_REG_ID_DEVICEID_Bits) -1) << ADS126X_REG_ID_DEVICEID_Pos)
			#define ADS126X_REG_ID_DEVICEID_Value_ADS1262							(0x00U << ADS126X_REG_ID_DEVICEID_Pos)
			#define ADS126X_REG_ID_DEVICEID_Value_ADS1263							(0x01U << ADS126X_REG_ID_DEVICEID_Pos)
			
			#define ADS126X_REG_ID_REVID_Pos													(0U)
			#define ADS126X_REG_ID_REVID_Bits													(5U)
			#define ADS126X_REG_ID_REVID_Msk													(((1U << ADS126X_REG_ID_REVID_Bits) -1) << ADS126X_REG_ID_REVID_Pos)
	
	#define ADS126X_REG_POWER_Addr			(0x01)			/* 	11h																																			reset													vbias			INTREF		*/
	//reset:	indicates ADC reset has occurred. Clear this bit to detect the next device reset
	//vbias:	enable the internal level shift voltage VBIAS = (Vavdd + Vavss)/2 to the AINCOM pin
	//INTREF:	enable the 2.5V internal voltage reference. note that the IDAC and temperature sensor require the internal voltage reference.
	
			#define ADS126X_REG_POWER_RESET_Pos												(4U)
			#define ADS126X_REG_POWER_RESET_Msk												(1U << ADS126X_REG_POWER_RESET_Pos)

			#define ADS126X_REG_POWER_VBIAS_Pos												(1U)
			#define ADS126X_REG_POWER_VBIAS_Msk												(1U << ADS126X_REG_POWER_VBIAS_Pos)

			#define ADS126X_REG_POWER_INTREF_Pos											(0U)
			#define ADS126X_REG_POWER_INTREF_Msk											(1U << ADS126X_REG_POWER_INTREF_Pos)

	#define ADS126X_REG_INTERFACE_Addr	(0x02)			/* 	05h																																								TIME_OUT	STATUS	| CRC[1:0]					|	*/
	// TIME_OUT:	enable Serial Interface Time-Out mode
	// STATUS:		enable the inclusion of the status byte during conversion data read-back
	// CRC[1:0]:	00: Checksum byte disabled; 01:Enable Checksum in Checksum mode during conversion data read-back(default); 10:Enable Checksum in CRC mode during conversion data readback; 11:reserved
	
			#define ADS126X_REG_INTERFACE_TIMEOUT_Pos									(3U)
			#define ADS126X_REG_INTERFACE_TIMEOUT_Msk									(1U << ADS126X_REG_INTERFACE_TIMEOUT_Pos)

			#define ADS126X_REG_INTERFACE_STATUS_Pos									(2U)
			#define ADS126X_REG_INTERFACE_STATUS_Msk									(1U << ADS126X_REG_INTERFACE_STATUS_Pos)
			
			#define ADS126X_REG_INTERFACE_CRC_Pos											(0U)
			#define ADS126X_REG_INTERFACE_CRC_Bits										(2U)
			#define ADS126X_REG_INTERFACE_CRC_Msk											(((1U << ADS126X_REG_INTERFACE_CRC_Bits) -1) << ADS126X_REG_INTERFACE_CRC_Pos)
			#define ADS126X_REG_INTERFACE_CRC_Value_DISABLED					(0x00U << ADS126X_REG_INTERFACE_CRC_Pos)
			#define ADS126X_REG_INTERFACE_CRC_Value_CHECKSUM					(0x01U << ADS126X_REG_INTERFACE_CRC_Pos)
			#define ADS126X_REG_INTERFACE_CRC_Value_CRC								(0x02U << ADS126X_REG_INTERFACE_CRC_Pos)
	
	#define ADS126X_REG_MODE0_Addr			(0x03)			/* 	00h				ADC1					Group1				REFREV			RUN_MODE	| CHOP[1:0]				|	DELAY[3:0]														|	*/
	//REFREV: 		reverses the ADC1 reference multiplexer output polarity 
	//RUNMODE:		select the ADC conversion (run) mode, 0: continuous conversion (default); 1:pulse conversion (one shot conversion)
	//CHOP[1:0]:	enable the ADC chop and IDAC rotation options  00:both disabled(default); 01:input chop enable; 10:IDAC rotation enable; 11:both enabled
	//DELAY[3:0]:	provides additional delay from conversion start to the beginning of the actual conversion. 0000: no delay(default); 0001: 8.7us; almost doubled for each increase of 1 up to 1011:8.8ms
	
			#define ADS126X_REG_MODE0_REFREV_Pos											(7U)
			#define ADS126X_REG_MODE0_REFREV_Msk											(1U << ADS126X_REG_MODE0_REFREV_Pos)
	
			#define ADS126X_REG_MODE0_RUNMODE_Pos											(6U)
			#define ADS126X_REG_MODE0_RUNMODE_Msk											(1U << ADS126X_REG_MODE0_RUNMODE_Pos)
			
			#define ADS126X_REG_MODE0_CHOPMODE_Pos										(4U)
			#define ADS126X_REG_MODE0_CHOPMODE_Bits										(2U)
			#define ADS126X_REG_MODE0_CHOPMODE_Msk										(((1U << ADS126X_REG_MODE0_CHOPMODE_Bits) -1) << ADS126X_REG_MODE0_CHOPMODE_Pos)
			#define ADS126X_REG_MODE0_CHOPMODE_Value_DISABLED					(0x00U << ADS126X_REG_MODE0_CHOPMODE_Msk)
			#define ADS126X_REG_MODE0_CHOPMODE_Value_INPUTCHOP				(0x01U << ADS126X_REG_MODE0_CHOPMODE_Msk)
			#define ADS126X_REG_MODE0_CHOPMODE_Value_IDACROTATION			(0x02U << ADS126X_REG_MODE0_CHOPMODE_Msk)
			#define ADS126X_REG_MODE0_CHOPMODE_Value_INPUTCHOP_IDACROTATION		(0x03U << ADS126X_REG_MODE0_CHOPMODE_Msk)
			
			
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Pos							(0U)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Bits						(4U)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Msk							(((1U << ADS126X_REG_MODE0_CONVERSIONDELAY_Bits) -1) << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_Zero			(0x00U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_8us7			(0x01U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_17us			(0x02U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_35us			(0x03U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_69us			(0x04U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_139us			(0x05U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_278us			(0x06U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_555us			(0x07U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_1ms1			(0x08U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_2ms2			(0x09U << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_4ms4			(0x0AU << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
			#define ADS126X_REG_MODE0_CONVERSIONDELAY_Value_8ms8			(0x0BU << ADS126X_REG_MODE0_CONVERSIONDELAY_Pos)
	
	#define ADS126X_REG_MODE1_Addr			(0x04)			/* 	80h				ADC1 					Group1			| FILTER[2:0]											| SBADC			SBPOL		| SBMAG[2:0]									| */
	//FILTER[2:0]:	000:Sinc1; 001:Sinc2; 010:Sinc3; 011:Sinc4; 100:FIR(default);
	//SBADC:	0:	sensor bias connected to ADC1 mux out (default); 1:sensor bias connected to ADC2 mux out
	//SBPOL:	more like control of current flow direction mode.	0:	sensor bias pull-up mode ( AINp pulled high, AINn pulled low, default);	1: sensor bias pull-down mode ( AINp pulled low, AINn pulled high )
	//SBMAG[2:0]:	sensor bias magnitude, 000: no current or resistor (default); 001: 0.5uA sensor bias current; 010:	2uA sensor bias current; 011: 2uA; 011:10uA; 100:50uA;101:200uA;110:10Mohm resistor
	// see page 34 - 9.3.3 Sensor Bias for details

			#define ADS126X_REG_MODE1_FILTER_Pos											(5U)
			#define ADS126X_REG_MODE1_FILTER_Bits											(3U)
			#define ADS126X_REG_MODE1_FILTER_Msk											(((1U << ADS126X_REG_MODE1_FILTER_Bits) -1) << ADS126X_REG_MODE1_FILTER_Pos)
			#define ADS126X_REG_MODE1_FILTER_Value_SINC1							(0x00U << ADS126X_REG_MODE1_FILTER_Pos)
			#define ADS126X_REG_MODE1_FILTER_Value_SINC2							(0x01U << ADS126X_REG_MODE1_FILTER_Pos)
			#define ADS126X_REG_MODE1_FILTER_Value_SINC3							(0x02U << ADS126X_REG_MODE1_FILTER_Pos)
			#define ADS126X_REG_MODE1_FILTER_Value_SINC4							(0x03U << ADS126X_REG_MODE1_FILTER_Pos)
			#define ADS126X_REG_MODE1_FILTER_Value_FIR								(0x04U << ADS126X_REG_MODE1_FILTER_Pos)

			#define ADS126X_REG_MODE1_SENSORBIASADC_Pos								(4U)
			#define ADS126X_REG_MODE1_SENSORBIASADC_Msk								(1U << ADS126X_REG_MODE1_SENSORBIASADC_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASADC_Value_ADC1				(0x00U << ADS126X_REG_MODE1_SENSORBIASADC_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASADC_Value_ADC2				(0x01U << ADS126X_REG_MODE1_SENSORBIASADC_Pos)
			
			#define ADS126X_REG_MODE1_SENSORBIASPOL_Pos								(3U)
			#define ADS126X_REG_MODE1_SENSORBIASPOL_Msk								(1U << ADS126X_REG_MODE1_SENSORBIASPOL_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASPOL_Value_PULLUP			(0x00U << ADS126X_REG_MODE1_SENSORBIASPOL_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASPOL_Value_PULLDOWN		(0x01U << ADS126X_REG_MODE1_SENSORBIASPOL_Pos)
			
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Pos								(0U)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Bits							(3U)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Msk								(((1U << ADS126X_REG_MODE1_SENSORBIASMAG_Bits) -1) << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_Zero				(0x00U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_uA5					(0x01U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_2uA					(0x02U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_10uA				(0x03U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_50uA				(0x04U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_200uA				(0x05U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)
			#define ADS126X_REG_MODE1_SENSORBIASMAG_Value_10Mohm			(0x06U << ADS126X_REG_MODE1_SENSORBIASMAG_Pos)

	#define ADS126X_REG_MODE2_Addr			(0x05)			/* 	04h				ADC1 					Group1				BYPASS			|	GAIN[2:0]										| DR[3:0]																| */
	//BYPASS: selects PGA bypass mode;
	//GAIN[2:0]:	select the PGA gain from 1 to 32
	//DR[3:0]:	select ADC data rate from 2.5sps to 384000sps

			#define ADS126X_REG_MODE2_BYPASS_Pos											(7U)
			#define ADS126X_REG_MODE2_BYPASS_Msk											(1U << ADS126X_REG_MODE2_BYPASS_Pos)

			#define ADS126X_REG_MODE2_GAIN_Pos												(4U)
			#define ADS126X_REG_MODE2_GAIN_Bits												(3U)
			#define ADS126X_REG_MODE2_GAIN_Msk												(((1U << ADS126X_REG_MODE2_GAIN_Bits) -1) << ADS126X_REG_MODE2_GAIN_Pos)
			#define ADS126X_REG_MODE1_GAIN_Value_1X										(0x00U << ADS126X_REG_MODE2_GAIN_Pos)
			#define ADS126X_REG_MODE1_GAIN_Value_2X										(0x01U << ADS126X_REG_MODE2_GAIN_Pos)
			#define ADS126X_REG_MODE1_GAIN_Value_4X										(0x02U << ADS126X_REG_MODE2_GAIN_Pos)
			#define ADS126X_REG_MODE1_GAIN_Value_8X										(0x03U << ADS126X_REG_MODE2_GAIN_Pos)
			#define ADS126X_REG_MODE1_GAIN_Value_16X									(0x04U << ADS126X_REG_MODE2_GAIN_Pos)
			#define ADS126X_REG_MODE1_GAIN_Value_32X									(0x05U << ADS126X_REG_MODE2_GAIN_Pos)

			#define ADS126X_REG_MODE2_DATARATE_Pos										(0U)
			#define ADS126X_REG_MODE2_DATARATE_Bits										(4U)
			#define ADS126X_REG_MODE2_DATARATE_Msk										(((1U << ADS126X_REG_MODE2_DATARATE_Bits) -1) << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_2SPS5						(0x00U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_5SPS							(0x01U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_10SPS						(0x02U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_16SPS						(0x03U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_20SPS						(0x04U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_50SPS						(0x05U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_60SPS						(0x06U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_100SPS						(0x07U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_400SPS						(0x08U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_1200SPS					(0x09U << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_2400SPS					(0x0AU << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_4800SPS					(0x0BU << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_7200SPS					(0x0CU << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_14400SPS					(0x0DU << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_19200SPS					(0x0EU << ADS126X_REG_MODE2_DATARATE_Pos)
			#define ADS126X_REG_MODE1_DATARATE_Value_38400SPS					(0x0FU << ADS126X_REG_MODE2_DATARATE_Pos)


	#define ADS126X_REG_INPUTMUX_Addr		(0x06)			/* 	01h				ADC1					Group1			| MUXP[3:0]																	|	MUXN[3:0]															| */
	//MUXP[3:0]	positive input multiplexer, from 0000 AIN0 (default) to AIN9, AINCOM, temperature positive, analog power positive, digital power positive, TDAC test signal positive, and 1111:float/open connection
	//MUXP[3:0]	negative input multiplexer, from 0000 AIN0, 0001 AIN1 (default) to AIN9, AINCOM, temperature negative, analog power negative, digital power negative, TDAC test signal negative, and 1111:float/open connection
	
			#define ADS126X_REG_INPUTMUX_MUXP_Pos										(4U)
			#define ADS126X_REG_INPUTMUX_MUXP_Bits									(4U)
			#define ADS126X_REG_INPUTMUX_MUXP_Msk										(((1U << ADS126X_REG_INPUTMUX_MUXP_Bits) -1) << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN0						(0x00U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN1						(0x01U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN2						(0x02U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN3						(0x03U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN4						(0x04U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN5						(0x05U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN6						(0x06U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN7						(0x07U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN8						(0x08U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AIN9						(0x09U << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_AINCOM					(0x0AU << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_TEMPP						(0x0BU << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_ANALOGPOWERMONITORP		(0x0CU << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_DIGITALPOWERMONITORP	(0x0DU << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_TDACP						(0x0EU << ADS126X_REG_INPUTMUX_MUXP_Pos)
			#define ADS126X_REG_INPUTMUX_MUXP_Value_FLOAT						(0x0FU << ADS126X_REG_INPUTMUX_MUXP_Pos)
	
			#define ADS126X_REG_INPUTMUX_MUXN_Pos										(0U)
			#define ADS126X_REG_INPUTMUX_MUXN_Bits									(4U)
			#define ADS126X_REG_INPUTMUX_MUXN_Msk										(((1U << ADS126X_REG_INPUTMUX_MUXN_Bits) -1) << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN0						(0x00U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN1						(0x01U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN2						(0x02U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN3						(0x03U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN4						(0x04U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN5						(0x05U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN6						(0x06U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN7						(0x07U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN8						(0x08U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AIN9						(0x09U << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_AINCOM					(0x0AU << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_TEMPN						(0x0BU << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_ANALOGPOWERMONITORN		(0x0CU << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_DIGITALPOWERMONITORN	(0x0DU << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_TDACN						(0x0EU << ADS126X_REG_INPUTMUX_MUXN_Pos)
			#define ADS126X_REG_INPUTMUX_MUXN_Value_FLOAT						(0x0FU << ADS126X_REG_INPUTMUX_MUXN_Pos)
	
	
	#define ADS126X_REG_OFCAL0_Addr			(0x07)			/* 	00h																	|	OFC[7:0]																																					| */
	#define ADS126X_REG_OFCAL1_Addr			(0x08)			/* 	00h																	|	OFC[15:8]																																					| */
	#define ADS126X_REG_OFCAL2_Addr			(0x09)			/* 	00h																	|	OFC[23:16]																																				| */
	//offset calibration, twos complement format, internally left-shifted to align with the 32-bit conversion result.
	
	#define ADS126X_REG_FSCAL0_Addr			(0x0A)			/* 	00h																	|	FSC[7:0]																																					| */
	#define ADS126X_REG_FSCAL1_Addr			(0x0B)			/* 	00h																	|	FSC[15:8]																																					| */
	#define ADS126X_REG_FSCAL2_Addr			(0x0C)			/* 	40h																	|	FSC[23:16]																																				| */
	//full-scale calibration, ADC divides this straight 24-bit binary value by 0x400000 to derive the gain coefficient.
	
	#define ADS126X_REG_IDACMUX_Addr		(0x0D)			/* 	bbh				ADC1 					Group1			|	MUX2[3:0]																	|	MUX1[3:0]															| */
	//MUX2[3:0]:	IDAC2 output multiplexer, selects the analog input pin to connect IDAC2 (0000 AIN0 to 1001 AIN9 1010 AINCOM, 1011 no connection(default);
	//MUX1[3:0]:	IDAC1 output multiplexer, selects the analog input pin to connect IDAC1 (0000 AIN0 to 1001 AIN9 1010 AINCOM, 1011 no connection(default);
	
			#define ADS126X_REG_IDACMUX_IDAC2_Pos										(4U)
			#define ADS126X_REG_IDACMUX_IDAC2_Bits									(4U)
			#define ADS126X_REG_IDACMUX_IDAC2_Msk										(((1U << ADS126X_REG_IDACMUX_IDAC2_Bits) -1) << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN0						(0x00U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN1						(0x01U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN2						(0x02U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN3						(0x03U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN4						(0x04U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN5						(0x05U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN6						(0x06U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN7						(0x07U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN8						(0x08U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AIN9						(0x09U << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_AINCOM					(0x0AU << ADS126X_REG_IDACMUX_IDAC2_Pos)
			#define ADS126X_REG_IDACMUX_IDAC2_Value_NOCONNECTION		(0x0BU << ADS126X_REG_IDACMUX_IDAC2_Pos)
	
			#define ADS126X_REG_IDACMUX_IDAC1_Pos										(0U)
			#define ADS126X_REG_IDACMUX_IDAC1_Bits									(4U)
			#define ADS126X_REG_IDACMUX_IDAC1_Msk										(((1U << ADS126X_REG_IDACMUX_IDAC1_Bits) -1) << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN0						(0x00U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN1						(0x01U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN2						(0x02U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN3						(0x03U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN4						(0x04U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN5						(0x05U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN6						(0x06U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN7						(0x07U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN8						(0x08U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AIN9						(0x09U << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_AINCOM					(0x0AU << ADS126X_REG_IDACMUX_IDAC1_Pos)
			#define ADS126X_REG_IDACMUX_IDAC1_Value_NOCONNECTION		(0x0BU << ADS126X_REG_IDACMUX_IDAC1_Pos)
	
	
	#define ADS126X_REG_IDACMAG_Addr		(0x0E)			/* 	00h				ADC1 					Group1			|	MAG2[3:0]																	|	MAG1[3:0]															| */
	//MAG2[3:0]:	IDAC2 current magnitude, 0000 off(default); 0001 50uA; almost doubled each after
	//MAG1[3:0]:	IDAC1 current magnitude, 0000 off(default); 0001 50uA; almost doubled each after
	
			#define ADS126X_REG_IDACMAG_IDAC2_Pos										(4U)
			#define ADS126X_REG_IDACMAG_IDAC2_Bits									(4U)
			#define ADS126X_REG_IDACMAG_IDAC2_Msk										(((1U << ADS126X_REG_IDACMAG_IDAC2_Bits) -1) << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_OFF							(0x00U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_50uA						(0x01U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_100uA						(0x02U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_250uA						(0x03U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_500uA						(0x04U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_750uA						(0x05U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_1000uA					(0x06U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_1500uA					(0x07U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_2000uA					(0x08U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_2500uA					(0x09U << ADS126X_REG_IDACMAG_IDAC2_Pos)
			#define ADS126X_REG_IDACMAG_IDAC2_Value_3000uA					(0x0AU << ADS126X_REG_IDACMAG_IDAC2_Pos)
	
			#define ADS126X_REG_IDACMAG_IDAC1_Pos										(0U)
			#define ADS126X_REG_IDACMAG_IDAC1_Bits									(4U)
			#define ADS126X_REG_IDACMAG_IDAC1_Msk										(((1U << ADS126X_REG_IDACMAG_IDAC1_Bits) -1) << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_OFF							(0x00U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_50uA						(0x01U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_100uA						(0x02U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_250uA						(0x03U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_500uA						(0x04U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_750uA						(0x05U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_1000uA					(0x06U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_1500uA					(0x07U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_2000uA					(0x08U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_2500uA					(0x09U << ADS126X_REG_IDACMAG_IDAC1_Pos)
			#define ADS126X_REG_IDACMAG_IDAC1_Value_3000uA					(0x0AU << ADS126X_REG_IDACMAG_IDAC1_Pos)
			
	#define ADS126X_REG_REFMUX_Addr			(0x0F)			/* 	00h				ADC1 					Group1															|	RMUXP[2:0]									|	RMUXN[2:0]									| */
	//RMUXP[2:0]:	reference positive input, 000: internal 2.5V reference positive(default); 001: AIN0, 010: AIN2; 011: AIN4; 100: Internal analog supply (Vavdd)
	//RMUXN[2:0]:	reference negative input, 000: internal 2.5V reference positive(default); 001: AIN1, 010: AIN3; 011: AIN5; 100: Internal analog supply (Vavss)
	
			#define ADS126X_REG_REFMUX_P_Pos												(3U)
			#define ADS126X_REG_REFMUX_P_Bits												(3U)
			#define ADS126X_REG_REFMUX_P_Msk												(((1U << ADS126X_REG_REFMUX_P_Bits) -1) << ADS126X_REG_REFMUX_P_Pos)
			#define ADS126X_REG_REFMUX_P_Value_INTERNAL							(0x00U << ADS126X_REG_REFMUX_P_Pos)
			#define ADS126X_REG_REFMUX_P_Value_AIN0									(0x01U << ADS126X_REG_REFMUX_P_Pos)
			#define ADS126X_REG_REFMUX_P_Value_AIN2									(0x02U << ADS126X_REG_REFMUX_P_Pos)
			#define ADS126X_REG_REFMUX_P_Value_AIN4									(0x03U << ADS126X_REG_REFMUX_P_Pos)
			#define ADS126X_REG_REFMUX_P_Value_ANALOGSUPPLYP				(0x04U << ADS126X_REG_REFMUX_P_Pos)
	
			#define ADS126X_REG_REFMUX_N_Pos												(0U)
			#define ADS126X_REG_REFMUX_N_Bits												(3U)
			#define ADS126X_REG_REFMUX_N_Msk												(((1U << ADS126X_REG_REFMUX_N_Bits) -1) << ADS126X_REG_REFMUX_N_Pos)
			#define ADS126X_REG_REFMUX_N_Value_INTERNAL							(0x00U << ADS126X_REG_REFMUX_N_Pos)
			#define ADS126X_REG_REFMUX_N_Value_AIN1									(0x01U << ADS126X_REG_REFMUX_N_Pos)
			#define ADS126X_REG_REFMUX_N_Value_AIN3									(0x02U << ADS126X_REG_REFMUX_N_Pos)
			#define ADS126X_REG_REFMUX_N_Value_AIN5									(0x03U << ADS126X_REG_REFMUX_N_Pos)
			#define ADS126X_REG_REFMUX_N_Value_ANALOGSUPPLYN				(0x04U << ADS126X_REG_REFMUX_N_Pos)
	
	#define ADS126X_REG_TDACP_Addr			(0x10)			/*	00h																		OUTP														|	MAGP[4:0]																				|	*/
	//OUTP: connects TDACP (test DAC positive) output to pin AIN6;
	//MAGP[4:0]: select the TDACP output magnitude. 01001:4.5V; 01000:3.5V; 00111:3V; 00110:2.75V; 00101:2.625V; 00100:2.5625V; ...; 10111:2V; 11000:1.5V; 11001:0.5V
	
			#define ADS126X_REG_TDACP_OUTP_Pos											(7U)
			#define ADS126X_REG_TDACP_OUTP_Msk											(1U << ADS126X_REG_TDACP_OUTP_Pos)
			
			#define ADS126X_REG_TDACP_MAGP_Pos											(0U)
			#define ADS126X_REG_TDACP_MAGP_Bits											(5U)
			#define ADS126X_REG_TDACP_MAGP_Msk											(((1U << ADS126X_REG_TDACP_MAGP_Bits) -1) << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_4V5								(0x09U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_3V5								(0x08U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_3V									(0x07U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V75								(0x06U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V625							(0x05U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V5625							(0x04U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V53125						(0x03U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V515625						(0x02U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V5078125					(0x01U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V5								(0x00U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V4921875					(0x11U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V484375						(0x12U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V46875						(0x13U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V4375							(0x14U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V375							(0x15U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V25								(0x16U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_2V									(0x17U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_1V5								(0x18U << ADS126X_REG_TDACP_MAGP_Pos)
			#define ADS126X_REG_TDACP_MAGP_Value_0V5								(0x19U << ADS126X_REG_TDACP_MAGP_Pos)
			
	
	#define ADS126X_REG_TDACN_Addr			(0x11)			/*	00h																		OUTN														|	MAGN[4:0]																				| */
	//OUTN: connects TDACN (test DAC negative) output to pin AIN7;
	//MAGP[4:0]: select the TDACP output magnitude. 01001:4.5V; 01000:3.5V; 00111:3V; 00110:2.75V; 00101:2.625V; 00100:2.5625V; ...; 10111:2V; 11000:1.5V; 11001:0.5V

			#define ADS126X_REG_TDACN_OUTN_Pos											(7U)
			#define ADS126X_REG_TDACN_OUTN_Msk											(1U << ADS126X_REG_TDACN_OUTN_Pos)
			
			#define ADS126X_REG_TDACN_MAGN_Pos											(0U)
			#define ADS126X_REG_TDACN_MAGN_Bits											(5U)
			#define ADS126X_REG_TDACN_MAGN_Msk											(((1U << ADS126X_REG_TDACN_MAGN_Bits) -1) << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_4V5								(0x09U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_3V5								(0x08U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_3V									(0x07U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V75								(0x06U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V625							(0x05U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V5625							(0x04U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V53125						(0x03U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V515625						(0x02U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V5078125					(0x01U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V5								(0x00U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V4921875					(0x11U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V484375						(0x12U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V46875						(0x13U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V4375							(0x14U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V375							(0x15U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V25								(0x16U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_2V									(0x17U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_1V5								(0x18U << ADS126X_REG_TDACN_MAGN_Pos)
			#define ADS126X_REG_TDACN_MAGN_Value_0V5								(0x19U << ADS126X_REG_TDACN_MAGN_Pos)

	#define ADS126X_REG_GPIOCON_Addr		(0x12)			/*	00h																	|	CON[7:0]																																					|	*/
	//GPIO connection register: connect GPIO[x] to corresponding AINx or not 
	
	#define ADS126X_REG_GPIODIR_Addr		(0x13)			/*	00h																	| DIR[7:0]																																					|	*/
	//GPIO direction register: 0: GPIO[x] is an output (default); 1:  GPIO[x] is an input;
	
	#define ADS126X_REG_GPIODAT_Addr		(0x14)			/*	00h																	|	DAT[7:0]																																					|	*/
	//GPIO Data register:	read return zero if GPIO[x] is configured as an output; for input, write sets the register value only (cannot set a input).
	
/* Additional ADS1263 Registers */
#ifdef ADS1263
	#define ADS126X_REG_ADC2CFG_Addr		(0x15)			/*	00h				ADC2					Group2			|	DR2[1:0]							|	REF2[2:0]										|	GAIN2[2:0]									| */
	//DR2[1:0]:		ADC2 data rate, 00:10sps; 01:100sps;10:400sps;11:800sps
	//REF2[2:0]:	ADC2 reference input 000:Internal 2.5V positive and negative(default); 001:AIN0 as positive and AIN1 negative; 010:AIN2 and AIN3; 011:AIN4 and AIN5; 100: internal Vavdd and Vavss
	//GAIN2[2:0]:	000: 1; 001:2; 010:4; 011:8; 100:16; 101:32;110:64; 111:128;
	
	#define ADS126X_REG_ADC2MUX_Addr		(0x16)			/*	01h				ADC2					Group2			|	MUXP2[3:0]																|	MUXN2[3:0]														| */
	//ADC2 input multiplexer reguister
	
	#define ADS126X_REG_ADC2OFC0_Addr		(0x17)			/*	00h																	|	OFC2[7:0]																																					|	*/
	#define ADS126X_REG_ADC2OFC1_Addr		(0x18)			/*	00h																	|	OFC2[15:8]																																				|	*/
	//ADC2 offset calibration registers
	
	#define ADS126X_REG_ADC2FSC0_Addr		(0x19)			/*	00h																	|	FSC2[7:0]																																					|	*/
	#define ADS126X_REG_ADC2FSC1_Addr		(0x1A)			/*	00h																	|	FSC2[15:8]																																				|	*/
	//ADC2 full-scale calibration registers
	
#endif /* ADS1263 */

#ifdef ADS1263
	#define ADS126X_NUM_REG 						(0x1B)			/* ADS1263 has 27 registers */
#else
	#define ADS126X_NUM_REG 						(0x15)			/* ADS1262 has 21 registers */
#endif

#define ADS126X_DATA_CIRCULARBUF_SIZE (1<<16)

// empty if read_index == write_index;
// voerrun if write_index + 1 == read_index; 
// one element space (at read_index-1) is unusable.
typedef struct {
						uint32_t buffer[ADS126X_DATA_CIRCULARBUF_SIZE];
	volatile	uint32_t read_index;
	volatile 	uint32_t write_index;
						uint32_t overrun_count;
}ADS126X_CircularBuf_StructDef;

typedef struct {
	uint32_t is_ADC_running;
	volatile uint32_t is_SPI_dataReady;
	volatile uint32_t is_ADC_dataReady;
	uint32_t missing_ADC_dataReady_count;
	uint32_t missing_SPI_dataReady_count;
	uint32_t warning_ADC_status_count;
	uint32_t warning_ADC_checksum_count;
	SPI_HandleTypeDef	*phspi;
	uint8_t RegData[ADS126X_NUM_REG];
	uint8_t spi_rxbytes[ADS126X_NUM_REG];
	uint8_t spi_txbytes[ADS126X_NUM_REG];
	ADS126X_CircularBuf_StructDef data_circularBuf;
}ADS126X_StructDef;

/////////////////////////////////////////////////////////////////////////////////////
// ADS126X Global Variables
////////////////////////////////////////////////////////////////////////////////////
extern ADS126X_StructDef ads126x;


/////////////////////////////////////////////////////////////////////////////////////
// ADS126X Function Prototypes
////////////////////////////////////////////////////////////////////////////////////

void ADS126x_PowerUp(void);

void ADS126x_PowerDown(void);

void ADS126x_Init(void);

void ADS126x_ReadMultiRegister(uint32_t StartAddr, uint32_t NumRegs, uint8_t *pdata);

void ADS126x_WriteMultiRegister(uint32_t StartAddr, uint32_t NumRegs, uint8_t *pdata);

void ADS126x_Start(void);

void ADS126x_Stop(void);

void ADS126x_PushDatatoCircularBuf(void);

void ADS126x_GetDatafromCircularBuf(uint32_t* data_buf, uint32_t *length);

#endif /* ADS126X_H_ */
