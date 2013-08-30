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

//#include "core_cm3.h"
#include "reset.h"
#include "lpc17xx_nvic.h"

//#include "debug.h"

/**
 * @brief  Initiate a system reset request.
 *
 * Initiate a system reset request to reset the MCU
 */
static __INLINE void reset(void)
{
  SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
  __DSB();                                                     /* Ensure completion of memory access */
  while(1);                                                    /* wait until reset */
}

unsigned int go_to_reset (void)
{
   reset(); // reset ao sistema

   return 0; // tudo bem, sem erro
}
