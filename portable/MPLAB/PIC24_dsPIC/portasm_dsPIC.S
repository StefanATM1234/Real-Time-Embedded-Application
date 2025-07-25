/*
    FreeRTOS V6.0.4 - Copyright (C) 2010 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

        .global _vPortYield
		.extern _vTaskSwitchContext
		.extern uxCriticalNesting

_vPortYield:

		PUSH	SR						/* Save the SR used by the task.... */
		PUSH	W0						/* ....then disable interrupts. */
		MOV		#32, W0
		MOV		W0, SR
		PUSH	W1						/* Save registers to the stack. */
		PUSH.D	W2
		PUSH.D	W4
		PUSH.D	W6
		PUSH.D 	W8
		PUSH.D 	W10
		PUSH.D	W12
		PUSH	W14
		PUSH	RCOUNT
		PUSH	TBLPAG
		PUSH	ACCAL
		PUSH	ACCAH
		PUSH	ACCAU
		PUSH	ACCBL
		PUSH	ACCBH
		PUSH	ACCBU
		PUSH	DCOUNT
		PUSH	DOSTARTL
		PUSH	DOSTARTH
		PUSH	DOENDL
		PUSH	DOENDH
		
																						
		PUSH	CORCON
		PUSH	PSVPAG
		MOV		_uxCriticalNesting, W0		/* Save the critical nesting counter for the task. */
		PUSH	W0
		MOV		_pxCurrentTCB, W0			/* Save the new top of stack into the TCB. */
		MOV		W15, [W0]

		call 	_vTaskSwitchContext

		MOV		_pxCurrentTCB, W0			/* Restore the stack pointer for the task. */
		MOV		[W0], W15
		POP		W0							/* Restore the critical nesting counter for the task. */
		MOV		W0, _uxCriticalNesting
		POP		PSVPAG
		POP		CORCON
		POP		DOENDH
		POP		DOENDL
		POP		DOSTARTH
		POP		DOSTARTL
		POP		DCOUNT
		POP		ACCBU
		POP		ACCBH
		POP		ACCBL
		POP		ACCAU
		POP		ACCAH
		POP		ACCAL
		POP		TBLPAG
		POP		RCOUNT						/* Restore the registers from the stack. */
		POP		W14
		POP.D	W12
		POP.D	W10
		POP.D	W8
		POP.D	W6
		POP.D	W4
		POP.D	W2
		POP.D	W0
		POP		SR

        return

        .end
