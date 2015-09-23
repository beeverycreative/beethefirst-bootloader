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


#define EXTRUDER_0_HEATER_PORT          2        /* P2.4 */
#define EXTRUDER_0_HEATER_PIN           (1 << 4) /* P2.4 */
#define HEATED_BED_0_HEATER_PORT        2        /* P2.5 */
#define HEATED_BED_0_HEATER_PIN         (1 << 5) /* P2.5 */
#define EXTRUDER_0_FAN_PORT             2         /* P2.3 */
#define EXTRUDER_0_FAN_PIN              (1<<3)
#define BUZZER_PORT                     2         /* P2.2 PWM1[3] */
#define BUZZER_PIN                      (1 << 22) /* P2.2 PWM1[3] */
#define STEPPERS_RESET_PORT             0         /* P0.22 */
#define STEPPERS_RESET_PIN              (1 << 22) /* P0.22 */

#define extruder_heater_off() digital_write(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, LOW);
#define heated_bed_off() digital_write(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, LOW);
#define extruder_fan_off() digital_write(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, LOW);
#define buzzer_off() digital_write(BUZZER_PORT, BUZZER_PIN, LOW);
#define X_ENABLE_PORT                   1         /* P1.24 */
#define X_ENABLE_PIN                    (1 << 24) /* P1.24 */
#define X_STEP_PORT                     1         /* P1.20 */
#define X_STEP_PIN                      (1 << 20) /* P1.20 */
#define Y_ENABLE_PORT                   1         /* P1.28 */
#define Y_ENABLE_PIN                    (1 << 28) /* P1.28 */
#define Y_STEP_PORT                     1         /* P1.25 */
#define Y_STEP_PIN                      (1 << 25) /* P1.25 */
#define Z_ENABLE_PORT                   0         /* P0. 1 */
#define Z_ENABLE_PIN                    (1 <<  1) /* P0. 1 */
#define Z_STEP_PORT                     1         /* P1.29 */
#define Z_STEP_PIN                      (1 << 29) /* P1.29 */
#define E_ENABLE_PORT                   2         /* P2.10 */
#define E_ENABLE_PIN                    (1 << 10) /* P2.10 */
#define E_STEP_PORT                     0         /* P0.10 */
#define E_STEP_PIN                      (1 << 10) /* P0.10 */
#define x_disable() digital_write(X_ENABLE_PORT, X_ENABLE_PIN, 1)
#define x_step() digital_write(X_STEP_PORT, X_STEP_PIN, 1)
#define y_disable() digital_write(Y_ENABLE_PORT, Y_ENABLE_PIN, 1)
#define y_step() digital_write(Y_STEP_PORT, Y_STEP_PIN, 1)
#define z_disable() digital_write(Z_ENABLE_PORT, Z_ENABLE_PIN, 1)
#define z_step() digital_write(Z_STEP_PORT, Z_STEP_PIN, 1)
#define e_disable() digital_write(E_ENABLE_PORT, E_ENABLE_PIN, 1)
#define e_step() digital_write(E_STEP_PORT, E_STEP_PIN, 1)

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

#ifdef BTF_PLUS_BATT
	//pin_mode(E_STEP_PORT, E_STEP_PIN, OUTPUT);
#endif

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


