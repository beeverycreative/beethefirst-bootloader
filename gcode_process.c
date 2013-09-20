/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011-2013 BEEVC - Electronic Systems	*/
/* All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in
 the documentation and/or other materials provided with the
 distribution.
 * Neither the name of the copyright holders nor the names of
 contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include "serial.h"
#include "usb.h"

#include "sermsg.h"
#include "sersendf.h"
#include "timer.h"
#include "reset.h"
#include "sbl_iap.h"
//#include "lpcusb_type.h"
#include "system_LPC17xx.h"
//#include "lpc17xx_pinsel.h"
#include "gcode_process.h"

#define EXTRUDER_NUM_1  1
#define EXTRUDER_NUM_2  2
#define EXTRUDER_NUM_3  4

uint8_t extruders_on;
double extruder_1_speed;         // in RPM

uint32_t auto_prime_steps = 0;
uint32_t auto_reverse_steps = 0;
const double auto_prime_feed_rate = 18000;
const double auto_reverse_feed_rate = 18000;
double auto_prime_factor = 640;
double auto_reverse_factor = 640;

eParseResult process_gcode_command(void) {
	double backup_f;
	uint8_t axisSelected = 0;
	eParseResult result = PR_OK;

	if (next_target.seen_M) {

		switch (next_target.M) {
			case 114:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr(" \r\n");

				char sector114[FLASH_BUF_SIZE] = {0};
				int j114 = 0;
				unsigned *pmem114;
				pmem114 = SECTOR_13_START;
				int i = 1;

				while (j114 < 120) {
					sector114[j114] = next_target.filename [i];
					pmem114++;
					j114++;
					i++;
				}

				while (j114 < FLASH_BUF_SIZE) {
					sector114[j114] = *pmem114;
					pmem114++;
					j114++;
				}

				prepare_sector(13, 13, SystemCoreClock);
				erase_sector(13, 13, SystemCoreClock);

				prepare_sector(13, 13, SystemCoreClock);
				write_data( (unsigned)(SystemCoreClock/1000),
							(unsigned)(SECTOR_13_START),
							(unsigned)sector114,
							(unsigned)FLASH_BUF_SIZE);


				compare_data((unsigned)(SystemCoreClock/1000),
							(unsigned)(SECTOR_13_START),
							(unsigned)sector114,
							(unsigned)FLASH_BUF_SIZE);
				int ja = 0;
				while (ja < 120) {
					next_target.filename [ja] = '0';
					ja++;
				}
			}
			break;

			// M115 - report firmware version
			case 115:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);

				char firmversion[120];
				char *pmem115;

				pmem115 = SECTOR_13_START;

				for (int i = 0; i < 120; i++) {
					firmversion[i] = *pmem115;
					pmem115++;
				}

				serial_writestr(" ");
				firmversion[119]=0;
				serial_writestr(firmversion);
				serial_writestr("\r\n");
			}
			break;

			// M116 - report bootloader version
			case 116:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr(" 3.1.1\r\n");
			}
			break;

			//M117 - report machine serial number
			case 117:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);

				//try to read the unique ID - not working in this LCP Revision
				/*
				 serial_writestr(" ");
				 read_device_serial_number();
				 serial_writestr("\r\n");
				 */

				char serialnumber[10] = {0};
				char *pmem117;

				pmem117 = SECTOR_14_START;

				for (int i = 0; i < 10; i++) {
					serialnumber[9 - i] = *pmem117;
					pmem117++;
				}

				serial_writestr(" ");
				serial_writeblock(serialnumber, 10);
				serial_writestr("\r\n");
			}
			break;

			case 118:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr(" \r\n");

				char sector118[FLASH_BUF_SIZE] = {0};
				int j118 = 0;
				uint32_t aux = 0;
				unsigned *pmem118;
				pmem118 = SECTOR_14_START;

				aux = (next_target.T);

				if(aux < 0 ||aux > 2147483647){
					serial_writestr("serial number invalid\n");
					break;
				}

				/*int to char*/
				for (j118 = 0; j118 < 10; j118++) {
					sector118[j118] = (char) ((aux) % 10 + '0');
					aux = aux / 10;
					pmem118++;
				}

				while (j118 < FLASH_BUF_SIZE) {
					sector118[j118] = *pmem118;
					pmem118++;
					j118++;
				}

				prepare_sector(14, 14, SystemCoreClock);
				erase_sector(14, 14, SystemCoreClock);

			    prepare_sector(14, 14, SystemCoreClock);
				write_data( (unsigned)(SystemCoreClock/1000),
							(unsigned)(SECTOR_14_START),
							(unsigned)sector118,
							(unsigned)FLASH_BUF_SIZE);


				compare_data((unsigned)(SystemCoreClock/1000),
							(unsigned)(SECTOR_14_START),
							(unsigned)sector118,
							(unsigned)FLASH_BUF_SIZE);
			}
			break;

			//M609 - reset the printer
			case 609:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr("\r\n");

				go_to_reset(); // reinicia o sistema
			}
			break;

			// M620 - devolve a versão/nome da impressora
			case 620:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);

				serial_writestr(" BEETHEFIRST V1.0");
				serial_writestr("\r\n");
			}
			break;

			case 630:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr("\r\n");

				/*delay to let the ok message to be written in the fifo*/
				int a = 0;
				for(unsigned int i = 0; i < 1000; i++){
					 for(unsigned int jj = 0; jj < 100; jj++){
						 if (a == 0){
							a = 1;
							GPIO_ClearValue(1, (1 << 14));
						}else{
							a = 0;
							GPIO_SetValue(1, (1 << 14));
						}
					 }
				 }

				char write_state;
				char *pmem630;

				pmem630 = SECTOR_15_START;

				write_state = *pmem630;

				if (user_code_present() && (write_state == '0')) {
					USBHwConnect(FALSE);
					execute_user_code();
				} else
					serial_writestr("No firmware available\n");
			}
			break;

			case 650:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr("\r\n");

				bytes_to_transfer = next_target.A;

				//bytes to transfer is in the right flash memory range
				if((bytes_to_transfer < 1) || (bytes_to_transfer > 458752)){
					serial_writestr("invalid number of bytes to transfer\n");
					break;
				}

				if (bytes_to_transfer > 0) {
					number_of_bytes = 0;
					transfer_mode = 1;

					//change read_state variable to invalid
					char sector650[FLASH_BUF_SIZE];
					char *pmem650;
					char state = 0xFF;
					int j650 = 0;
					pmem650 = SECTOR_15_START;

					sector650[j650] = state;
					j650++;
					pmem650++;

					while (j650 < FLASH_BUF_SIZE) {
						sector650[j650] = *pmem650;
						pmem650++;
						j650++;
					}
/*
					prepare_sector(SECTOR_15_START, SECTOR_15_START, SystemCoreClock);
					erase_sector(SECTOR_15_START, SECTOR_15_START, SystemCoreClock);
					write_flash((unsigned *) (SECTOR_15_START), (char *) &sector650,
							FLASH_BUF_SIZE);
*/
					prepare_sector(15, 15, SystemCoreClock);
					erase_sector(15, 15, SystemCoreClock);

					prepare_sector(15, 15, SystemCoreClock);
					write_data( (unsigned)(SystemCoreClock/1000),
								(unsigned)(SECTOR_15_START),
								(unsigned)sector650,
								(unsigned)FLASH_BUF_SIZE);


					compare_data((unsigned)(SystemCoreClock/1000),
								(unsigned)(SECTOR_15_START),
								(unsigned)sector650,
								(unsigned)FLASH_BUF_SIZE);


					//apagar todos os sectores menos o último
					prepare_sector(16, 16, SystemCoreClock);
					erase_sector(16, 16, SystemCoreClock);

					prepare_sector(17, 17, SystemCoreClock);
					erase_sector(17, 17, SystemCoreClock);

					prepare_sector(18, 18, SystemCoreClock);
					erase_sector(18, 18, SystemCoreClock);

					prepare_sector(19, 19, SystemCoreClock);
					erase_sector(19, 19, SystemCoreClock);

					prepare_sector(20, 20, SystemCoreClock);
					erase_sector(20, 20, SystemCoreClock);

					prepare_sector(21, 21, SystemCoreClock);
					erase_sector(21, 21, SystemCoreClock);

					prepare_sector(22, 22, SystemCoreClock);
					erase_sector(22, 22, SystemCoreClock);

					prepare_sector(23, 23, SystemCoreClock);
					erase_sector(23, 23, SystemCoreClock);

					prepare_sector(24, 24, SystemCoreClock);
					erase_sector(24, 24, SystemCoreClock);

					prepare_sector(25, 25, SystemCoreClock);
					erase_sector(25, 25, SystemCoreClock);

					prepare_sector(26, 26, SystemCoreClock);
					erase_sector(26, 26, SystemCoreClock);

					prepare_sector(27, 27, SystemCoreClock);
					erase_sector(27, 27, SystemCoreClock);

					prepare_sector(28, 28, SystemCoreClock);
					erase_sector(28, 28, SystemCoreClock);

				} else
					serial_writestr("Error erasing user flash\n");
			}
			break;

			case 651:
			{
				//sends ok message
				serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);

				char write_state651;
				char *pmem651;

				pmem651 = SECTOR_15_START;

				write_state651 = *pmem651;

				serial_writestr(" ");
				serial_writechar(write_state651);
				serial_writestr("\r\n");
			}
			break;

			case 652:
			{
				//sends ok message
	/*			serial_writestr("ok ");
				serial_writestr(" N:");
				serwrite_uint32(next_target.N);
				serial_writestr("\r\n");
*/
				bytes_to_transfer = next_target.A;

				if((bytes_to_transfer < 1) || (bytes_to_transfer > 458752)){
					serial_writestr("invalid number of bytes to read\n");
					break;
				}

				/*delay to allow the ok message to be transmited*/
				for (int j = 0; j < 1000; j++) {
					for (int jj = 0; jj < 1000; jj++) {
						GPIO_ClearValue(1, (1 << 14));
					}
				}

				unsigned char *pmem652;
				pmem652 = (USER_FLASH_START);
				unsigned int po = 0;
				unsigned int full_b = bytes_to_transfer - (bytes_to_transfer%64);

				for( po = 0; po < full_b; po = po + 64) {

					while(fifo_free(&txfifo) < 64){
						GPIO_SetValue (1, (1<<14));
						GPIO_ClearValue (1, (1<<14));
					}

					serial_writeblock(pmem652 + po, 64);
				}
				while(fifo_free(&txfifo) < 64){
					GPIO_SetValue (1, (1<<14));
					GPIO_ClearValue (1, (1<<14));
				}
				serial_writeblock(pmem652+full_b, bytes_to_transfer%64);
				bytes_to_transfer = 0;
			}
			break;

			// unknown mcode: spit an error
			default:
			{
				serial_writestr("ok Bad M-code ");
				serwrite_uint32(next_target.M);
				serial_writestr("\r\n");
			}
		}
	}
	return result;
}
