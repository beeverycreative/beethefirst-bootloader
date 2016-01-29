/* Copyright (c) 2011, 2013 BEEVC - Electronic Systems	*/
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

#include "serial.h"
#include "serial_fifo.h"
#include "usb.h"
#include "LPC17xx.h"
#include "core_cm3.h"


void serial_init()
{
    USBSerial_Init();        
}

/*
        Read
*/

int serial_rxchars()
{
  return fifo_avail(&rxfifo);
}

uint8_t serial_popchar()
{
  uint8_t c = 0;

  fifo_get(&rxfifo, &c);

  return c;
}

/*
        Write
*/

uint8_t serial_txchars()
{
  return fifo_avail(&txfifo);
}

void serial_writechar(char data)
{
  fifo_put(&txfifo, data);
}

void serial_writeblock(void *data, int datalen)
{
  int i;

  for (i = 0; i < datalen; i++)
    serial_writechar(((uint8_t *) data)[i]);

}

void serial_writestr(char *data)
{
  uint8_t i = 0, r;

  while ((r = data[i++]))
    serial_writechar(r);

}
