#include <p33FJ256GP710.h>
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include <FreeRTOS.h>
#include <task.h>
#include <list.h>
#include <semphr.h>



_FOSC(OSCIOFNC_ON & FCKSM_CSDCMD & POSCMD_NONE);	//Oscillator Configuration
//_FOSCSEL(FNOSC_FRCPLL);							//Oscillator Selection PLL
_FOSCSEL(FNOSC_FRC);								//Oscillator Selection no PLL
_FWDT(FWDTEN_OFF);									//Turn off WatchDog Timer
_FGS(GCP_OFF);										//Turn off code protect
_FPOR(FPWRT_PWR1);									//Turn off power up timer

void Task1(void *params) {
	while(1) {		
		PORTBbits.RB15 = 1;
		vTaskDelay(500);
		PORTBbits.RB15 = 0;
		vTaskDelay(500);
	}
}

void Task2(void *params) {
	while(1) {				
		PORTBbits.RB14 = 1;
		vTaskDelay(1000);
		PORTBbits.RB14 = 0;
		vTaskDelay(1000);
	}
}

void Task3(void *params) {
	while(1) {		
		PORTBbits.RB13 = 1;
		vTaskDelay(200);
		PORTBbits.RB13 = 0;
		vTaskDelay(200);
	}
}

void Task4(void *params) {
	while(1) {				
		PORTBbits.RB12 = 1;
		vTaskDelay(100);
		PORTBbits.RB12 = 0;
		vTaskDelay(100);
	}
}

void vSerialTxISR()	{
}

void vSerialRxISR() {
}

int main() {
	ADPCFG = 0xFFFF;				//make ADC pins all digital
	
	/*
	_PLLDIV = 38;            		// M=40: PLL Feedback Divisor bits
	CLKDIV = 0;              		// N1=2: PLL VCO Output Divider Select bits
                           			// N2=2: PLL Phase Detector Input Divider bits

	RCONbits.SWDTEN = 0;      		// Disable Watch Dog Timer
	while(OSCCONbits.LOCK != 1);	// Wait for PLL to lock
	*/
	
	TRISB = 0;
	
	xTaskCreate(Task1, (signed portCHAR *) "Ts1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(Task2, (signed portCHAR *) "Ts2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(Task3, (signed portCHAR *) "Ts3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(Task4, (signed portCHAR *) "Ts4", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
	vTaskStartScheduler();

	while(1) {
		ClrWdt();
	}
	
	return 0;
}

void vApplicationIdleHook( void )
{
	// Schedule the co-routines from within the idle task hook.
	vCoRoutineSchedule();
}
//-----------------------------------------------------------*/

