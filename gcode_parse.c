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


#include	<string.h>
#include 	<stdbool.h>

#include	"serial.h"
#include	"sermsg.h"
#include	"gcode_process.h"
#include	"gcode_parse.h"

#include 	"sbl_config.h"
#include	"sbl_iap.h"

#include	"lpcusb_type.h"
#include 	"system_LPC17xx.h"

GCODE_COMMAND next_target;
uint32_t transfer_mode;
uint32_t bytes_to_transfer;

uint32_t number_of_bytes;
uint32_t start_address;

/*
	Switch user friendly values to coding friendly values

	This also affects the possible build volume. We have +-2^31 numbers available and as we internally measure position in steps and use a precision factor of 1000, this translates into a possible range of

		2^31 mm / STEPS_PER_MM_x / 1000

	for each axis. For a M6 threaded rod driven machine and 1/16 microstepping this evaluates to

		2^31 mm / 200 / 1 / 16 / 1000 = 671 mm,

	which is about the worst case we have. All other machines have a bigger build volume.
*/


static uint8_t last_field = 0;

#define crc(a, b)		(a ^ b)

static struct {
  uint8_t sign; 
  int exponent;
  } read_digit;
static double value;

// accept the next character and process it
void gcode_parse_char(uint8_t c);

/*
	utility functions
*/

double power (double x, int exp)
{  
  double result = 1.0;
  while (exp--)
  {
    result = result * x;
  }

  return result;
}

/*
	public functions
*/

eParseResult gcode_parse_line (tLineBuffer *pLine) 
{

  eParseResult result = PR_OK;

  for (int j=0; j < pLine->len; j++)
  {
	gcode_parse_char (pLine->data [j]);
  }

  // process
  result = process_gcode_command();

  // reset variables
  next_target.seen_X = next_target.seen_Y = next_target.seen_Z = \
  next_target.seen_E = next_target.seen_F = next_target.seen_S = \
  next_target.seen_P = next_target.seen_N = next_target.seen_M = \
  next_target.seen_checksum = next_target.seen_semi_comment = \
  next_target.seen_parens_comment = next_target.checksum_read = \
  next_target.seen_T = next_target.seen_A = next_target.checksum_calculated = 0;
  next_target.chpos = 0;
  next_target.N = 0;
  last_field = 0;
  read_digit.sign = read_digit.exponent = 0;
  value = 0;

  return result;
}

/****************************************************************************
*                                                                           *
* Character Received - add it to our command                                *
*                                                                           *
****************************************************************************/
void gcode_parse_char(uint8_t c) 
{
  uint8_t save_ch;

  #ifdef ASTERISK_IN_CHECKSUM_INCLUDED
  if (next_target.seen_checksum == 0)
  {
    next_target.checksum_calculated = crc(next_target.checksum_calculated, c);
  }
  #endif

  save_ch = c;

  // uppercase
  if (c >= 'a' && c <= 'z')
  {
    c &= ~32;
  }

  // process previous field
  if (last_field)
  {
    // check if we're seeing a new field or end of line
    // any character will start a new field, even invalid/unknown ones
    if ((c >= 'A' && c <= 'Z') || c == '*' || (c == 10) || (c == 13))
    {
      // before using value, apply the sign
      if (read_digit.sign)
      {
        value = -value;
      }

      switch (last_field)
      {
        case 'M':
        next_target.M = value;
        // this is a bit hacky since string parameters don't fit in general G code syntax
        // NB: filename MUST start with a letter and MUST NOT contain spaces
        // letters will also be converted to uppercase
        if ((next_target.M == 114)
        		|| next_target.M == 118){
          next_target.getting_string = 1;
        }
        break;


        case 'N':
        next_target.N = value;
        break;

        case 'T':
        next_target.T = value;
        break;

        case 'S':
        next_target.S = value;
        break;

        case 'A':
        next_target.A = value;
        break;

        case '*':
        next_target.checksum_read = value;
        break;
      }

      // reset for next field
      last_field = 0;
      read_digit.sign = read_digit.exponent = 0;
      value = 0;
    }
  }

  if (next_target.getting_string)
  {
    if ((c == 10) || (c == 13) || ( c == ' ')  || ( c == '*'))
    {
      next_target.getting_string = 0;
    }
    else
    {
      if (next_target.chpos < sizeof(next_target.filename))
      {
        next_target.filename [next_target.chpos++] = c;
        next_target.filename [next_target.chpos] = 0;
      }
    }      
  }

  // skip comments, filenames
  if (next_target.seen_semi_comment == 0 && next_target.seen_parens_comment == 0 && next_target.getting_string == 0)
  {
    // new field?
    if ((c >= 'A' && c <= 'Z') || c == '*')
    {
      last_field = c;
    }

    // process character
    switch (c)
    {
      case 'M':
      next_target.seen_M = 1;
      break;

      case 'N':
      next_target.seen_N = 1;
      break;

      case 'T':
      next_target.seen_T= 1;
      break;

      case 'A':
      next_target.seen_A = 1;
      break;

      case 'S':
      next_target.seen_S = 1;
      break;

      case '*':
      next_target.seen_checksum = 1;
      break;

      // comments
      case ';':
      case '^':
      next_target.seen_semi_comment = 1;
      break;

      case '(':
      next_target.seen_parens_comment = 1;
      break;

      // now for some numeracy
      case '-':
      read_digit.sign = 1;
      // force sign to be at start of number, so 1-2 = -2 instead of -12
      read_digit.exponent = 0;
      break;

      case '.':
      if (read_digit.exponent == 0)
      {
        read_digit.exponent = 1;
      }
      break;

      default:
      // can't do ranges in switch..case, so process actual digits here
      if (c >= '0' && c <= '9')
      {
        if (read_digit.exponent == 0)
        {
          value = value * 10 + (c - '0');
        }
        else
        {
          value += (double)(c - '0') / power(10, read_digit.exponent);
        }

        if (read_digit.exponent)
        {
          read_digit.exponent++;
        }
      }
    }
  }
  else if (next_target.seen_parens_comment == 1 && c == ')')
  {
    next_target.seen_parens_comment = 0; // recognize stuff after a (comment)
  }

  #ifndef ASTERISK_IN_CHECKSUM_INCLUDED
  if (next_target.seen_checksum == 0)
  {
    next_target.checksum_calculated = crc(next_target.checksum_calculated, save_ch);
  }
  #endif
}
