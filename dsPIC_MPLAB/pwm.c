#include "p33FJ128MC802.h"

void init_PWM1()
{
 P1TCONbits.PTOPS = 0; // Timer base output scale
 P1TCONbits.PTMOD = 0; // Free running
 P1TCONbits.PTCKPS = 0b11; // 11 PWM time base input clock period is 64 TCY (1:64 prescale)
 P1TMRbits.PTDIR = 0; // Numara in sus pana cand timerul = perioada
 P1TMRbits.PTMR = 0; // Baza de timp
					
					 
 P1DC3 = 1850;          //corespunzatoare 1.5 ms 
		//1250;//1 ms 2500;	//2 ms//temperaturii de 25°C ii va
					    // corespunde pozitia centrala 
				  
 P1TPER = 12500;        //20 ms 
						//PWM Time Base Period Register

 PWM1CON1bits.PMOD3 = 1; // Canalele PWM3H si PWM3L sunt independente

 PWM1CON1bits.PEN3H = 0; // Pinul PWM3H setat pe I/O general purpose
 PWM1CON1bits.PEN3L = 1; // Pinul PWM3L setat pe iesire PWM          ----> RB10

 PWM1CON2bits.UDIS = 1; // Disable Updates from duty cycle and period buffers
 /* Clock period for Dead Time Unit A is TcY */
 P1DTCON1bits.DTAPS = 0b00;
 /* Clock period for Dead Time Unit B is TcY */
 P1DTCON1bits.DTBPS = 0b00;

 /* Dead time value for Dead Time Unit A */
 P1DTCON1bits.DTA = 10;
 /* Dead time value for Dead Time Unit B */
 P1DTCON1bits.DTB = 20;
 /* Dead Time Unit selection for PWM signals */
 /* Dead Time Unit A selected for PWM active transitions */ 
 P1DTCON2bits.DTS3A = 0;
 //P1DTCON2bits.DTS2A = 0;
 //P1DTCON2bits.DTS1A = 0;

 /* Dead Time Unit B selected for PWM inactive transitions */
 P1DTCON2bits.DTS3I = 1;
 //P1DTCON2bits.DTS2I = 1;
 //P1DTCON2bits.DTS1I = 1;
 P1TCONbits.PTEN = 1; /* Enable the PWM Module */
}
