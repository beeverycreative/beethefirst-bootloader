/* Copyright (c) 2013 BEEVC - Electronic Systems	*/
/*
 * This file is part of BEESOFT software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. BEESOFT is
 * distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details. You should have received a copy of the
 * GNU General Public License along with BEESOFT. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

// CMSIS
#include	"LPC17xx.h"
#include	"core_cm3.h"

// NXP
#include	"lpc17xx_nvic.h"
#include	"lpc17xx_pinsel.h"
#include	"lpc17xx_gpio.h"
#include 	"lpc17xx_wdt.h"

// LPCUSB
#include	"usbapi.h"

// Local
#include	"sbl_iap.h"
#include	"sbl_config.h"
#include    "ios.h"
#include	"gcode_parse.h"
#include	"gcode_process.h"
#include 	"usb.h"
#include 	"serial.h"

#include 	"pinout.h"
#include 	"timer.h"
#include 	"adc.h"
#include 	"ExpBoard.h"

#ifdef EXP_BOARD
	double current_temp_r2c2 = 0;
	uint32_t adc_filtered_r2c2 = 4095;
	int32_t adc_r2c2_raw = 4095;
#ifdef USE_BATT
	int32_t ps_ext_state = false;
	int32_t battADC_raw = 4095;
	int32_t batt_filtered = 4095;
	bool charging = false;
#endif
#endif

bool debugMode = false;

//Bootlaoder Timer
tTimer bootloaderTimer;

void bootloaderTimerCallback (tTimer *pTimer)
{
#ifdef EXP_BOARD
	int32_t r2c2_buf[5];
	for(int32_t i = 0; i < 5; i++)
	{
		r2c2_buf[i] = analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
	}

	adc_r2c2_raw = getMedianValue(r2c2_buf);

	adc_filtered_r2c2 = adc_filtered_r2c2*0.9 + adc_r2c2_raw*0.1;

	double volts = (double) adc_filtered_r2c2*(3.3/4096);

	current_temp_r2c2 = (volts - 0.5)*100;


#ifdef USE_BATT
	ps_ext_state = digital_read(PS_EXT_READ_PORT,PS_EXT_READ_PIN);

	if(ps_ext_state)
	{
		int32_t batt_buf[5];
		for(int32_t j = 0; j < 5; j++)
		{
			batt_buf[j] = analog_read(BATT_ADC_SENSOR_ADC_CHANNEL);
		}

		battADC_raw = getMedianValue(batt_buf);

		batt_filtered = batt_filtered*0.9 + battADC_raw*0.1;

		if(current_temp_r2c2 > 45 || current_temp_r2c2 < 0)
		{
			STEP_uC_disable();
			charging = false;
		} else {
			STEP_uC_enable();
			charging = true;
		}
	} else {
		BATT_uC_disable();
	}

#endif
#endif
}

BOOL bootloader_button_pressed(void)
{
	/* Configure bootloader IO button P4.29 */
	PINSEL_CFG_Type pin;
	pin.Portnum = 4;
	pin.Pinnum = 29;
	pin.Funcnum = PINSEL_FUNC_0;
	pin.Pinmode = PINSEL_PINMODE_PULLUP;
	pin.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&pin);

	/* set as input */
	GPIO_SetDir(4, (1<<29), 0);

	/* Verify if bootloader pin is activated */
	if(GPIO_ReadValue(4) & (1<<29))
		return FALSE;
	return TRUE;
}

void io_init(void)
{
	/*
	 * Disable some pins like the ones for heaters while on bootloader, if not, the heaters would be ON
	 */
	/* Extruder 0 Heater pin */
	pin_mode(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, OUTPUT);
	extruder_heater_off();
	/* Heated Bed 0 Heater pin */
	pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, OUTPUT);
	heated_bed_off();
	/* Extruder fan pin */
	pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
	extruder_fan_off();
	/* Buzzer fan pin */
	pin_mode(BUZZER_PORT, BUZZER_PIN, OUTPUT);
	buzzer_off();
	/* Disable reset for all stepper motors */
	pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
	digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, 1);
	/* Disable all motors BUT enable step pin, so each LED will be ON */
	pin_mode(X_STEP_PORT, X_STEP_PIN, OUTPUT);
	pin_mode(X_ENABLE_PORT, X_ENABLE_PIN, OUTPUT);
	x_disable();
	x_step();
	pin_mode(Y_STEP_PORT, Y_STEP_PIN, OUTPUT);
	pin_mode(Y_ENABLE_PORT, Y_ENABLE_PIN, OUTPUT);
	y_disable();
	y_step();
	pin_mode(Z_STEP_PORT, Z_STEP_PIN, OUTPUT);
	pin_mode(Z_ENABLE_PORT, Z_ENABLE_PIN, OUTPUT);
	z_disable();
	z_step();
	pin_mode(E_STEP_PORT, E_STEP_PIN, OUTPUT);
	pin_mode(E_ENABLE_PORT, E_ENABLE_PIN, OUTPUT);
	e_disable();
	e_step();

#ifndef EXP_BOARD
	pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
	extruder_fan_off();
#endif

#ifdef EXP_BOARD
	pin_mode(FAN_EXT_ON_PORT, FAN_EXT_ON_PIN, OUTPUT);
	extruder_block_fan_off();

	pin_mode(BW_ON_PORT,BW_ON_PIN, OUTPUT);
	pin_mode(BW_V1_PORT,BW_V1_PIN, OUTPUT);
	blower_off();

	pin_mode(BW_ON_PORT,BW_ON_PIN, OUTPUT);
	blower_off();

	pin_mode(R2C2_FAN_PORT,R2C2_FAN_PIN, OUTPUT);
	r2c2_fan_off();
#endif

#ifdef USE_BATT
	//Battery Digital I/Os
	pin_mode(PS_EXT_READ_PORT,PS_EXT_READ_PIN,INPUT);

	pin_mode(STEP_uC_ON_PORT, STEP_uC_ON_PIN, OUTPUT);
	STEP_uC_enable();
	pin_mode(BATT_uC_ON_PORT, BATT_uC_ON_PIN, OUTPUT);
	BATT_uC_disable();
#endif
}

void adc_init(void)
{
#ifdef EXP_BOARD
	PINSEL_CFG_Type PinCfg;

	//R2C2 TEMPERATURE ADC CONFIG
	PinCfg.Funcnum = PINSEL_FUNC_1; /*ADC Function*/
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Portnum = R2C2_TEMP_ADC_PORT;
	PinCfg.Pinnum = R2C2_TEMP_ADC_PIN;
	PINSEL_ConfigPin(&PinCfg);

#ifdef USE_BATT
  //Battery ADC CONFIG

  PinCfg.Funcnum = PINSEL_FUNC_1;
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = BATT_ADC_PORT;
  PinCfg.Pinnum = BATT_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

#endif

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
#endif
}

void init_analogVars(void)
{
#ifdef EXP_BOARD

  __disable_irq();
  adc_filtered_r2c2 = analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
  adc_filtered_r2c2 += analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
  adc_filtered_r2c2 += analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
  adc_filtered_r2c2 /= 3;
  __enable_irq();

#ifdef USE_BATT
  __disable_irq();
  batt_filtered = analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
  batt_filtered += analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
  batt_filtered += analog_read(R2C2_TEMP_SENSOR_ADC_CHANNEL);
  batt_filtered /= 3;
  __enable_irq();
#endif
#endif
}
int main()
{
	eParseResult parse_result;

	// DeInit NVIC and SCBNVIC
	NVIC_DeInit();
	NVIC_SCBDeInit();

	/* Configure the NVIC Preemption Priority Bits:
	 * two (2) bits of preemption priority, six (6) bits of sub-priority.
	 * Since the Number of Bits used for Priority Levels is five (5), so the
	 * actual bit number of sub-priority is three (3)
	 */
	NVIC_SetPriorityGrouping(0x05);

	NVIC_SetVTOR(0x00000000);

	SysTickTimer_Init(); // Initialize the timer for millis()

	//delay_ms(1000);

	//Config IOs
	io_init();

	//Config ADCs
	adc_init();

	//Initialize Analog Variables
	init_analogVars();

	//Bootloader Timer
	AddSlowTimer (&bootloaderTimer);
	StartSlowTimer (&bootloaderTimer, 50, bootloaderTimerCallback);
	bootloaderTimer.AutoReload = 1;

	char write_state;
	char *pmem630;

	USBSerial_Init();

	/*variables used to store the firmware in the flash memory*/
	unsigned char sector[FLASH_BUF_SIZE] = {0};
	unsigned char *pmem;
	pmem = (USER_FLASH_START);
	unsigned int counter = 0;
	unsigned result;

	WDT_Init (WDT_CLKSRC_PCLK, WDT_MODE_RESET );
	WDT_Start (30000000);

	// main loop
	for (;;)
	{
		WDT_Feed();

		// process characters from the serial port
		while (!serial_line_buf.seen_lf
				&& ((serial_rxchars() != 0)
						|| (transfer_mode==1))
						&& (serial_line_buf.len < MAX_LINE)){

			/*if it is transfer but it has no characteres, it should just continue*/
			if(serial_rxchars() != 0){

				unsigned char c = serial_popchar();

				serial_line_buf.data [serial_line_buf.len] = c;
				serial_line_buf.len++;

				/*if it is the last character and is not in transfer mode*/
				if (((c == 10)
						|| (c == 13))
						&& (transfer_mode == 0)){

					if (serial_line_buf.len > 1){
						serial_line_buf.seen_lf = 1;
					}else{
						serial_line_buf.len = 0;
					}
				}/*no need for else*/

				if (transfer_mode == 1){
					number_of_bytes = number_of_bytes + 1;
					if (number_of_bytes == bytes_to_transfer){
						serial_line_buf.seen_lf = 1;
						break;
					}/*no need for else*/
				}/*no need for else*/
			}/*no need for else*/
		}

		if(!transfer_mode
				&& (serial_line_buf.len != 0)){

			parse_result = gcode_parse_line (&serial_line_buf);
			serial_line_buf.len = 0;
			serial_line_buf.seen_lf = 0;
		}/*no need for else*/

		if(transfer_mode
				&& (serial_line_buf.len != 0)){

			/*used in the debug loop back*/
			serial_writeblock(serial_line_buf.data,serial_line_buf.len);

			/*This should never occur!*/
			if (!((counter + serial_line_buf.len) <= FLASH_BUF_SIZE)){
				serial_writestr("Danger: sector overflow\n");
			}/*no need for else*/

			/*the USB message is transfered to the array that is going to be stored*/
			for(int i = 0; i < serial_line_buf.len; i++){
				sector[counter+i] = serial_line_buf.data[i];
			}

			counter = counter + serial_line_buf.len;
			serial_line_buf.len = 0;
			serial_line_buf.seen_lf = 0;

			/* number_of_bytes ==  -> last message*/
			if (number_of_bytes == bytes_to_transfer){

				/*fill the rest of the array*/
				for(;counter < FLASH_BUF_SIZE; counter++){
					sector[counter] = 255;
				}
			}/*no need for else*/

			/*if the array to be written is full, it is write*/
			if (counter == FLASH_BUF_SIZE){

				result = write_flash(pmem, sector, FLASH_BUF_SIZE);
				if(result != CMD_SUCCESS){
					serial_writestr("Problem writing sector\n");
				}/*No need for else*/

				pmem = pmem + FLASH_BUF_SIZE;
				counter = 0;

				if (number_of_bytes == bytes_to_transfer){

					//change write_state variable to invalid
					char sector1[FLASH_BUF_SIZE];
					char *pmem1;
					char state = '0';
					int j1 = 0;
					pmem1 = SECTOR_15_START;

					sector1[j1] = state;
					j1++;
					pmem1++;
					while (j1 < FLASH_BUF_SIZE) {
						sector1[j1] = *pmem1;
						pmem1++;
						j1++;
					}

					prepare_sector(15, 15, SystemCoreClock);
					erase_sector(15, 15, SystemCoreClock);

					prepare_sector(15, 15, SystemCoreClock);
					write_data( (unsigned)(SystemCoreClock/1000),
							(unsigned)(SECTOR_15_START),
							(unsigned)sector1,
							(unsigned)FLASH_BUF_SIZE);


					compare_data((unsigned)(SystemCoreClock/1000),
							(unsigned)(SECTOR_15_START),
							(unsigned)sector1,
							(unsigned)FLASH_BUF_SIZE);

					bytes_to_transfer = 0;
					number_of_bytes = 0;
					transfer_mode = 0;
					pmem = (USER_FLASH_START);

				}/*no need for else*/
			}/*no need for else*/
		}/*no need for else*/
	}
}


