// UARTIntsTestMain.c
// Runs on LM4F120/TM4C123
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// September 12, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Program 5.11 Section 5.6, Program 3.10

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

#include <stdint.h>
#include <string.h>
#include "PLL.h"
#include "UART.h"
#include "ADC.h"
#include "ST7735.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

unsigned short data;
uint32_t channel;
uint32_t rate;
uint32_t sample_number;

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}

void interpreter (char * command){
	if(strcmp(command, "lcd") == 0){		
		UART_OutString("-----You select LCD -----");
		OutCRLF();
	  ST7735_FillScreen(0xFFFF);            // set screen to white
    ST7735_Message (1, 1, "hello", 24);
	} else if(strcmp(command,"adc") == 0){
		UART_OutString("Please enter the channel 0-11");
		OutCRLF();
		channel = UART_InUDec();
		OutCRLF();
		UART_OutString("Please enter the sampling rates(Hz) 100-10000");
		OutCRLF();
		rate = UART_InUDec();
		OutCRLF();
		UART_OutString("Please enter the number of samples");
		OutCRLF();
		sample_number = UART_InUDec();
		OutCRLF();
		if(sample_number == 1){
			ADC_Open(0);
			data = ADC_In();
			UART_OutString("-----ADC Result -----");
			OutCRLF();
			UART_OutUDec(data);
		}else{
			unsigned short buffer[sample_number];
			ADC_Collect(channel,rate,buffer,sample_number);
		  while(ADC_Status()){}
			UART_OutString("-----ADC Result -----");
			for(int i = 0; i < sample_number; i++){
				UART_OutString("sample ");
				UART_OutUDec(sample_number+1);
				UART_OutChar(':');
				UART_OutUDec(buffer[sample_number]);
				OutCRLF();
			}
		}
	} else if(strcmp(command,"timer") == 0){
		
	}
}
//debug code
int main(void){
  char string[20];  // global to assist in debugging
  PLL_Init();               // set system clock to 50 MHz
	ST7735_InitR(INITR_REDTAB); // initialize LCD
  UART_Init();              // initialize UART
  OutCRLF();
  UART_OutChar('-');
  UART_OutChar('-');
  UART_OutChar('>');

  while(1){
    UART_OutString("Pleast input the command");
		OutCRLF();
    UART_InString(string,19);
    UART_OutString(" OutString="); UART_OutString(string); OutCRLF();
		interpreter (string);
  }
}
