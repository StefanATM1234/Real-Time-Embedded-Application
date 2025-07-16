/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

/* Demo application includes. */
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"

/* Own includes. */
#include "new_lcd.h"
#include "new_serial.h"
#include "libq.h"


/* ************************* */
#include "adcDrv1.h"
#include "ds18s20.h"
#include "pwm.h"
#include "new_lcd.h"
#include "timertest.h"

/* Demo task priorities. */
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY				( 2 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( portTickType ) 3000 / portTICK_RATE_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* Baud rate used by the comtest tasks. */
//#define mainCOM_TEST_BAUD_RATE				( 19200 )
#define mainCOM_TEST_BAUD_RATE				( 9600 )

// Definire lungime coada UART1
#define comBUFFER_LEN						( 200 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( portTickType ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( portTickType ) 0xffff )

/* The LED used by the comtest tasks.  mainCOM_TEST_LED + 1 is also used.
See the comtest.c file for more information. */
#define mainCOM_TEST_LED					( 6 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 300 )



// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC);
// Enable Clock Switching and Configure
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);		// FRC + PLL
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);		// XT + PLL
_FWDT(FWDTEN_OFF); 		// Watchdog Timer Enabled/disabled by user software


static void prvSetupHardware( void );

/* The queue used to send messages to the LCD task. */
static xQueueHandle xUART1_Queue;

/*------------------------------------------------------------------------*/

/*VARIABILE GLOBALE*/

float valoareTemperatura;  //Variabila globala in care va fi salvata valoarea 
                          //citita de la senzorul de temperaura 


xTaskHandle ht1, ht2, ht3, ht4, ht5;

unsigned int flag=1;	  // FLAG STARE APLICATIE 
unsigned int mod_lucru=1; //flag mod lucru
/*-----------------------------------------------------------------------*/

void __attribute__ ((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
	if( flag == 1)					 
	{
		flag = 0;                    //	ca sa reporneasca aplicatia o sa imi reporneasca taskurile

		xTaskResumeFromISR(ht2);	  //pus in stare de asteptare 
		xTaskResumeFromISR(ht3);	  //pus in stare de asteptare 
		xTaskResumeFromISR(ht4);      //pus in stare de asteptare 
		xTaskResumeFromISR(ht5);      //pus in stare de asteptare

		IFS0bits.INT0IF = 0;          /*Resetem flagul coresp. intreruperii INT0*/

	}
	else
	{	flag = 1;        			   // 
		IFS0bits.INT0IF = 0;          /*Resetem flagul coresp. intreruperii INT0*/
	} 
}

// Task T1 - Control general al aplicatiei
// Rol: gestioneazã pornirea/opririea aplicatiei pe baza variabilei globale `flag`
// - Dacã aplicatia e activã: aprinde intermitent un LED pe pinul RB11
// - Dacã aplicatia e opritã: suspendã celelalte taskuri (T2-T5)
void T1(void *params) {          //verfica flagul si suspenda toate task-urile 

	for (;;){							      
	if(flag == 1)				 // aplicatie pornita
	{
		_RB11=~_RB11;    			 //ledul de la portul RB11 va fi aprins intermitent
	vTaskDelay(500);
	}
	else						//ca sa imi opreasca aplicatia le suspenda pe toate
	{  
		vTaskSuspend(ht2);   
		vTaskSuspend(ht3);
		vTaskSuspend(ht4);
		vTaskSuspend(ht5);
				
		}
		vTaskDelay(100);
	}
}

void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt( void )
{	
	if(flag ==1)     //aplicatia pornita
	{ 
		_RB11=1;        //led APRINS
	}
	else
	{
		_RB11 = ~_RB11;    //led aprins intermitent
	}
 	IFS0bits.T1IF = 0; // Resetare flag de intrerupere
} 

// Task_Temperatura - Achizi?ia valorii de temperaturã
// Acest task cite?te în mod periodic temperatura de la senzorul DS18S20
// ?i actualizeazã variabila globalã `valoareTemperatura`.
// Frecven?a de citire este de 1 secundã (1000ms)

void Task_Temperatura(void *params) 
{ 
	for (;;)
	{                                     
		valoareTemperatura = ds1820_read();		//achizitionare valoare temperatura de la senzor

		vTaskDelay(1000);
	}

}

// Task Mod_lucru - Controleazã modul de func?ionare al sistemului: automat sau manual
// În func?ie de valoarea variabilei globale `mod_lucru`, se genereazã un semnal PWM
// pentru comanda unui servomotor ce ac?ioneazã o trapã.
void Mod_lucru(void *params)
{
		unsigned int temp;
		unsigned int tens;
	for(;;)
	{
		
			if(mod_lucru == 1)          // mod de lucru automat                                        
			{	
				_RB1 = 1;				//led Aprins pentru mod automat
			 	temp =(((((valoareTemperatura*1000)-20000)/100))*10)+1350;  // valoarea factorului de umplere
				P1DC3 = temp;                                               //registrul duty cycle pwm 
			
			}
			else                        // mod de lucru manual
			{ 
				_RB1 = 0;				//led Stins pentru mod manual
				if(valoareTensiune<=1)
				{
					P1DC3=1250;			//valoare P1DC3 
				}
				else
				{
				tens= ((((valoareTensiune*1000)-1000)/20)*10)+1350;         // valoarea factorului de umplere
				P1DC3 = tens;                                                //registrul duty cycle 
				}
			}
			
		vTaskDelay(1000);	
		}
}

// Task_afisare_lcd - Afisare informatii sistem pe LCD 4x20
// Rol: oferã utilizatorului feedback vizual asupra functionãrii sistemului:
// - temperatura cititã, modul de lucru, tensiunea de la potentiometru, ultima comandã

void Task_afisare_lcd(void *params)
{
	unsigned char val[10];
	for(;;){
	LCD_line(1);
	LCD_printf("Temp:");
	LCD_Goto(1,15);
	_itoaQ15(valoareTemperatura,val);  //conversie din integer in ASCII
	LCD_printf(val);
	LCD_line(2);
	LCD_printf("Mod_lucru:");
	LCD_Goto(2,13);
	if(mod_lucru==0)
	{
		LCD_printf("Automat");
	}
	else 
	{ 
		LCD_printf("Manual");
	}

	LCD_line(3);
	LCD_printf("Tensiune");
	LCD_Goto(3,15);
	_itoaQ15(valoareTensiune,val);   //face conversia de la integer la caractere string
	LCD_printf(val);
	LCD_line(4);
	LCD_printf("Ultima_com");
	LCD_Goto(4,15);
	LCD_printf("c");

	vTaskDelay(1000);
	
	}		
}


/*Task-uri interfata seriala UART1 */
void T5(void *params) {

		unsigned char cByteRxed;
		unsigned char val[10];
		
	for (;;)
		{	
		
			vSerialPutString(NULL,"MENIU COMENZI \n",0);  									
	 		
			if(xSerialGetChar(NULL, &cByteRxed, 0xffff))
				{
		
					if(cByteRxed == '1')
					{ 
 						if(mod_lucru == 0)
						{
						mod_lucru=1; //AUTOMAT
						}
					
					else 
					{
						mod_lucru=0; //MANUAL
					}

					}
					if(cByteRxed == '2')
					{
						if(mod_lucru == 0)
						{
							vSerialPutString(NULL,"Mod Automat\n",0);     //returneaza Mod Automat
						}
						else 
						{
							vSerialPutString(NULL,"Mod Manual\n",0);     //returneaza Mod Manual
						}
						
					}
					if(cByteRxed =='3')
					{
						vSerialPutString(NULL,"Temperatura:",0);  //returneaza temperatura pe seriala
						_itoaQ15(valoareTemperatura,val);       // face conversia din integer cu ASCII
						vSerialPutString(NULL,val,0);     
						 
					}
				
					
				}
			
		}
}

/* ---------------------------------------------------------------------*/

int main(void)
{
    // Ini?ializarea hardware-ului aplica?iei (PLL, ADC, PWM, LCD, UART, întreruperi etc.)
    prvSetupHardware();

    // Creare task T1 – Control general aplica?ie (verificã flag ?i suspendã/reporne?te restul taskurilor)
    xTaskCreate(T1,                             // Func?ia taskului
        (signed portCHAR *)"Ts1",      // Nume simbolic
        configMINIMAL_STACK_SIZE,      // Dimensiunea minimã a stivei
        NULL,                          // Parametru (nu e folosit)
        tskIDLE_PRIORITY + 5,          // Prioritate mare (pentru a supraveghea celelalte taskuri)
        &ht1                           // Handle pentru referin?ã la suspendare/resume
    );

    // Task T2 – Citire temperaturã de la senzor DS18S20
    xTaskCreate(Task_Temperatura, "Ts2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &ht2);

    // Task T3 – Afi?are pe LCD (temperaturã, mod lucru, tensiune)
    xTaskCreate(Task_afisare_lcd, "Ts3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &ht3);

    // Task T4 – Controlul PWM pentru trapã, în func?ie de mod_lucru
    xTaskCreate(Mod_lucru, "Ts4", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &ht4);

    // Task T5 – Comunicare serialã (UART1) – prime?te comenzi ?i transmite rãspunsuri
    xTaskCreate(T5, "Ts5", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, &ht5);

    // Lansarea sistemului de operare FreeRTOS – începe execu?ia multitasking
    vTaskStartScheduler();
    return 0;
}

/*-----------------------------------------------------------*/

void initPLL(void)
{
// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 41; 		// M = 43 FRC
	//PLLFBD = 30; 		// M = 32 XT
	CLKDIVbits.PLLPOST=0; 	// N1 = 2
	CLKDIVbits.PLLPRE=0; 	// N2 = 2

// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);	// FRC
	//__builtin_write_OSCCONH(0x03);	// XT
	__builtin_write_OSCCONL(0x01);

// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b001);	// FRC
	//while (OSCCONbits.COSC != 0b011);	// XT

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};
}

void Init_Timer4( void )
{
 T1CON = 0;
 IPC0bits.T1IP = 1;
 IEC0bits.T1IE = 1;
 T1CONbits.TCS = 0;
 T1CONbits.TGATE = 0;
 T1CONbits.TCKPS = 0b11; // Selectare prescaler 1:256
 TMR1= 0x0000;
 PR1 = 14740;  //trebuie calculata ?
 T1CONbits.TON = 1;
}

static void prvSetupHardware( void )
{
	// Disable Watch Dog Timer
	RCONbits.SWDTEN=0;
	ADPCFG = 0xFFFF;				//make ADC pins all digital - adaugat
	vParTestInitialise();

	// Peripheral Initialisation
	initPLL();

	// Setam pinii RB3 si RB7
	 TRISB=0x0000;  //setam toti pinii ca iesire 
	_TRISB2 = 1;  //Pinul RB2 este setat ca intrare(comunicarea cu senzorul de temperatura) 
	_TRISB3 = 1;  //Pinul RB3 este setat ca intrare (pentru conversia AD)
	_TRISB1 = 1;   //Pinul RB1 este setat ca intrare 
	_TRISB7 = 1;  //Pinul RB7 este setat ca intrare (intreruperea INT0)
	_RB10 = 0;    //Pinul RB10  SETAT CA IESIRE --> PWM-RB10
	 PORTB=0x0000;  //seteaza valoare pentru pinii setati ca iesire
	
	_INT0IF = 0; 		/*Resetem flagul coresp. intreruperii INT0*/
	_INT0IE = 1; 		/* Se permite lucrul cu întreruperea INT0 */
	_INT0EP = 1;        /* Se stabileste pe ce front se generaza INT0 */
	
	init_PWM1();                // Initializare PWM 
   	initAdc1();             	// Initializare ADC
	initTmr3();					// Initializare TIMER 3
	
//	Init_Timer4();              // Configurare Timer 2
	
	
	init_temperatura();        //initializare temperatura
	

 	// Initializare LCD
	LCD_init(); 


    // Initializare interfata seriala UART1
	xSerialPortInitMinimal( mainCOM_TEST_BAUD_RATE, comBUFFER_LEN );
        
}


void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/

