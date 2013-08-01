// CMSIS
#include	"LPC17xx.h"
#include	"core_cm3.h"

// NXP
#include	"lpc17xx_nvic.h"
#include	"lpc17xx_pinsel.h"
#include	"lpc17xx_gpio.h"

// FatFs
#include	"ff.h"

// LPCUSB
#include	"usbapi.h"

// Local
#include	"blockdev.h"
#include	"sbl_iap.h"
#include	"sbl_config.h"
#include	"msc_scsi.h"
#include	"spi.h"
//#include	"uart.h"
#include	"debug.h"
#include    "ios.h"
#include	"gcode_parse.h"
#include	"gcode_process.h"
#include 	"usb.h"
#include 	"planner.h"
//#include	"r2c2.h"
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

	/*configuration of debug pins*/
	/* NOT USED*/
/*
	pin_mode(1, (1<<9), 1);
	pin_mode(1, (1<<10), 1);
	pin_mode(1, (1<<14), 1);
*/
	USBSerial_Init();

	/*variables used to store the firmware in the flash memory*/
	unsigned char sector[FLASH_BUF_SIZE] = {0};
	unsigned char *pmem;
	pmem=(USER_FLASH_START);
	unsigned int counter = 0;

	// main loop
	for (;;)
	{
		// process characters from the serial port
		while (!serial_line_buf.seen_lf && ((serial_rxchars() != 0) || (transfer_mode==1)) && (serial_line_buf.len < MAX_LINE)){
			if(serial_rxchars() != 0){

				unsigned char c= serial_popchar();
				serial_line_buf.data [serial_line_buf.len] = c;
				serial_line_buf.len++;

				/*if it is the last character and is not in transfer mode*/
				if (((c==10) || (c==13)) && (transfer_mode==0)){
					if (serial_line_buf.len > 1){
						serial_line_buf.seen_lf = 1;
					}else{
						serial_line_buf.len = 0;
					}
				}

				if (transfer_mode==1){
					number_of_bytes = number_of_bytes + 1;
					if (number_of_bytes == bytes_to_transfer){
						serial_line_buf.seen_lf = 1;
						break;
					}
				}
			}
		}

		if(!transfer_mode && (serial_line_buf.len != 0)){
			parse_result = gcode_parse_line (&serial_line_buf);
			serial_line_buf.len = 0;
			serial_line_buf.seen_lf = 0;
		}

		if(transfer_mode && (serial_line_buf.len != 0)){

			/*used in the debug loop back*/
			/*serial_writeblock(serial_line_buf.data,serial_line_buf.len);*/

			//This should never occur!
			if (!((counter + serial_line_buf.len) <= FLASH_BUF_SIZE)){
				serial_writestr("Danger: sector overflow\n");
			}

			/*the USB message is transfered to the array that is going to be stored*/
			for(int i = 0; i < serial_line_buf.len; i++){
				sector[counter+i]= serial_line_buf.data[i];
			}

			counter = counter + serial_line_buf.len;
			serial_line_buf.len = 0;
			serial_line_buf.seen_lf = 0;

			// number_of_bytes == bytes_to_transfer -> last message
			if (number_of_bytes == bytes_to_transfer){
				GPIO_SetValue (1, (1<<10));

				/*fill the rest of the array*/
				for(;counter<FLASH_BUF_SIZE;counter++){
					sector[counter] = 255;
				}
				counter = FLASH_BUF_SIZE;
				transfer_mode = 0;
			}

			/*if the array to be written is full, it is write*/
			if (counter == FLASH_BUF_SIZE){
				if (number_of_bytes == bytes_to_transfer){
					GPIO_SetValue (1, (1<<9));
				}

				write_flash((pmem), sector, FLASH_BUF_SIZE);
				pmem=pmem+FLASH_BUF_SIZE;
				counter = 0;
			}

			/*reads the firmware and sends it to the software so that it can check that it is well written*/
			/*NOT USED*/
			/*
			if (number_of_bytes == bytes_to_transfer){

				int jj=1;
				for(int j=0;j<1000;j++){

					for(int jjj=0;jjj<1000;jjj++){

						GPIO_ClearValue (1, (1<<14));
					}
				}

				 unsigned char  *pmem2;
				 pmem2 = (unsigned *) (USER_FLASH_START);
				 unsigned int j=0;
				 unsigned int full_b = bytes_to_transfer - (bytes_to_transfer%64);

				 for( j = 0; j < full_b; j = j + 64){
					for(unsigned int i=0;i<1000;i++){
						GPIO_SetValue (1, (1<<14));

						for(unsigned int jjj=0;jjj<100;jjj++){

							GPIO_ClearValue (1, (1<<14));
						}
					}
					serial_writeblock(pmem2+j, 64);
				 }
				 serial_writeblock(pmem2+full_b, bytes_to_transfer%64);
			}
			*/
		}
	}
}


