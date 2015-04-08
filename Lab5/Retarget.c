#include <stdio.h>
#include <rt_misc.h>
#include "inc/tm4c123gh6pm.h"
#include "UART.h"
#include "eFile.h"

#pragma import(__use_no_semihosting_swi)

extern int StreamToFile;

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc (int ch, FILE *f) {
 if(StreamToFile){
	if(eFile_Write(ch)){ // close file on error
		eFile_EndRedirectToFile(); // cannot write to file
		return 1; // failure
	}
	return 0; // success writing
 }
 // regular UART output
 UART_OutChar(ch);
 return 0;
}

int fgetc (FILE *f){ 
  return (UART_InChar()); 
} 

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int c) {
  UART_OutChar(c);
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}

