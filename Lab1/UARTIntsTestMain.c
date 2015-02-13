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
#include "OS.h"
#include "inc/tm4c123gh6pm.h"


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

unsigned short data;
uint32_t channel;
uint32_t rate;
uint32_t sample_number;
unsigned short buffer[100];
unsigned short device;
unsigned short line;
char lcd_string[20];
long value;
unsigned int timer_period;

//Initialize PORT 0
#define PIND0                    (*((volatile unsigned long *)0x40007004))
void GPIO_Init(void){  
	volatile unsigned long delay;
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;    // activate port F
	delay=SYSCTL_RCGC2_R; 									 // waiting for the hardware to setup
	GPIO_PORTF_DIR_R |= 0x04;                // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;             // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;                // enable digital I/O on PF2
                                           // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;                  // disable analog functionality on PF
  GPIO_PORTF_DATA_R &= ~0x04;              // turn off LED
}
//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}
void dummy(void){
	  //printf("dummy\r\n");
	  //GPIO_PORTF_DATA_R ^= 0x04;             // toggle LED
}
void interpreter (char * command){
	if(strcmp(command, "lcd") == 0){		
		printf("-----You select LCD -----\r\n");
		printf("-----Please Enter the device(top/down:0/1)-----\r\n");
		device = UART_InUDec();
		OutCRLF();
		printf("-----Line 0 to 3-----\r\n");
		line = UART_InUDec();
		OutCRLF();
		printf("-----String-----\r\n");
		UART_InString(lcd_string,20);
		OutCRLF();
		printf("-----Value-----\r\n");
		value = UART_InUDec();
		OutCRLF();
    ST7735_Message (device, line, lcd_string, value);
	} else if(strcmp(command,"adc") == 0){
		printf("Please enter the channel 0-11\r\n");
		channel = UART_InUDec();
		OutCRLF();
		printf("Please enter the sampling rates(Hz) 100-10000\r\n");
		rate = UART_InUDec();
		OutCRLF();
		printf("Please enter the number of samples\r\n");
		sample_number = UART_InUDec();
		OutCRLF();
		if(sample_number == 1){
			ADC_Open(0);
			data = ADC_In();
			printf("ADC Result: %d\r\n",data);
		}else{
			ADC_Collect(channel,rate,buffer,sample_number);
		  while(ADC_Status()){};
			printf("-----ADC Result -----\r\n");
			for(int i = 0; i < sample_number; i++){
				printf("sample result[%d]:%d\r\n",i+1,buffer[i]);
			}
			//printf("hello\r\n");
		}
	} else if(strcmp(command,"timer") == 0){
		 printf("Please enter the system timer period\r\n");
		 timer_period  = UART_InUDec();
     OS_AddPeriodicThread(&dummy,timer_period,1);
		 while(1){
			 GPIO_PORTF_DATA_R ^= 0x04;
		 }
		 //printf("hello\r\n");
	}
}

//debug code
int main(void){
  char string[20];  // global to assist in debugging
  PLL_Init();               // set system clock to 50 MHz
	GPIO_Init();
	ST7735_InitR(INITR_REDTAB); // initialize LCD
	ST7735_FillScreen(0xFFFF); // set screen to white
  UART_Init();              // initialize UART
  while(1){
    printf("Pleast input the command\r\n");
    UART_InString(string,19);
    OutCRLF();
		interpreter (string);
		//ST7735_Message (0, 0, "hello", 1);
			
  }	
}
