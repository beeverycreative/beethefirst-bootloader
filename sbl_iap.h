//-----------------------------------------------------------------------------
// Software that is described herein is for illustrative purposes only  
// which provides customers with programming information regarding the  
// products. This software is supplied "AS IS" without any warranties.  
// NXP Semiconductors assumes no responsibility or liability for the 
// use of the software, conveys no license or title under any patent, 
// copyright, or mask work right to the product. NXP Semiconductors 
// reserves the right to make changes in the software without 
// notification. NXP Semiconductors also make no representation or 
// warranty that such application will be suitable for the specified 
// use without further testing or modification. 
//-----------------------------------------------------------------------------

#ifndef  _SBL_IAP_H
#define  _SBL_IAP_H

extern const unsigned sector_start_map[];
extern const unsigned sector_end_map[];

void read_device_serial_number(void);
unsigned write_flash(unsigned * dst, char * src, unsigned no_of_bytes);
void execute_user_code(void);
int user_code_present(void);
void check_isp_entry_pin(void);
void find_prepare_sector(unsigned cclk, unsigned dst);
void find_erase_sector(unsigned cclk, unsigned dst);
void prepare_sector(unsigned start_sector,unsigned end_sector,unsigned cclk);
void erase_sector(unsigned start_sector, unsigned end_sector, unsigned cclk);
void compare_data(unsigned cclk, unsigned dst,unsigned  flash_data_buf, unsigned count);
void write_data(unsigned cclk, unsigned dst, unsigned flash_data_buf, unsigned count);


static unsigned param_table[5];
static unsigned result_table[5];

typedef enum
{
	PREPARE_SECTOR_FOR_WRITE	=50,
	COPY_RAM_TO_FLASH			=51,
	ERASE_SECTOR				=52,
	BLANK_CHECK_SECTOR			=53,
	READ_PART_ID				=54,
	READ_BOOT_VER				=55,
	COMPARE                     =56,
	REINVOKE_ISP				=57,
	READ_DEVICE_SERIAL			=58
}IAP_Command_Code;

#define CMD_SUCCESS 0
#define IAP_ADDRESS 0x1FFF1FF1

#endif /* _SBL_IAP_H */
