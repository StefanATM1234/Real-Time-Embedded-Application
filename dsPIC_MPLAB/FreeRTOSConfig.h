/*
    FreeRTOS V6.0.4 - Copyright (C) 2010 Real Time Engineers Ltd.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

//#include <p33FJ256GP710.h>
#include <p33FJ128MC802.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK				0
#define configUSE_TICK_HOOK				0
#define configTICK_RATE_HZ				( ( portTickType ) 1000 )
#define configCPU_CLOCK_HZ				( ( unsigned long ) 40000000 )  /* Fosc / 2 */
//#define configCPU_CLOCK_HZ				( ( unsigned long ) 3685000 )  /* Fosc / 2 */
#define configMAX_PRIORITIES			( ( unsigned portBASE_TYPE ) 4 )
#define configMINIMAL_STACK_SIZE		( 105 )
#define configTOTAL_HEAP_SIZE			( ( size_t ) 5120 )
#define configMAX_TASK_NAME_LEN			( 4 )
#define configUSE_TRACE_FACILITY		0
#define configUSE_16_BIT_TICKS			1
#define configIDLE_SHOULD_YIELD			1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		1
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete				0
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1


#define configKERNEL_INTERRUPT_PRIORITY	0x01


#endif /* FREERTOS_CONFIG_H */
