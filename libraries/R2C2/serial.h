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

#ifndef	_SERIAL_H
#define	_SERIAL_H

#include <stdint.h>


// initialise serial subsystem
void serial_init(void);

// return number of characters in the receive buffer, and number of spaces in the send buffer
int serial_rxchars(void);
uint8_t serial_txchars(void);

// read one character
uint8_t serial_popchar(void);
// send one character
void serial_writechar(char data);

// read/write many characters
// uint8_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(void *data, int datalen);

void serial_writestr(char *data);

#endif	/* _SERIAL_H */
