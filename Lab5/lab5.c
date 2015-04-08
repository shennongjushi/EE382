//*****************************************************************************
//
// Lab5.c - user programs, File system, stream data onto disk
// Jonathan Valvano, March 16, 2011, EE345M
//     You may implement Lab 5 without the oLED display
//*****************************************************************************
// PF1/IDX1 is user input select switch
// PE1/PWM5 is user input down switch 
#include <stdio.h>
#include <string.h>
#include "inc/hw_types.h"
//#include "serial.h"
//#include "adc.h"
#include "OS.h"
#include "inc/tm4c123gh6pm.h"
#include "edisk.h"
#include "efile.h"
#include "ADC.h"
#include "ADC_SW.h"
#include "UART.h"
#include "PLL.h"

unsigned long NumCreated;   // number of foreground threads created
unsigned long NumSamples;   // incremented every sample
unsigned long DataLost;     // data sent by Producer, but not received by Consumer

int Running;                // true while robot is running

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units

#define GPIO_PF0  (*((volatile unsigned long *)0x40025004))
#define GPIO_PF1  (*((volatile unsigned long *)0x40025008))
#define GPIO_PF2  (*((volatile unsigned long *)0x40025010))
#define GPIO_PF3  (*((volatile unsigned long *)0x40025020))
#define GPIO_PG1  (*((volatile unsigned long *)0x40026008))
// PF1/IDX1 is user input select switch
// PE1/PWM5 is user input down switch 
// PF0/PWM0 is debugging output on Systick
// PF2/LED1 is debugging output 
// PF3/LED0 is debugging output 
// PG1/PWM1 is debugging output 

#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))

void PortE_Init(void){ unsigned long volatile delay;
  //SYSCTL_RCGC2_R |= 0x30;       // activate port E
	SYSCTL_RCGCGPIO_R |= 0x10;
  /*delay = SYSCTL_RCGC2_R;        
  delay = SYSCTL_RCGC2_R;    
	delay = SYSCTL_RCGC2_R;	*/
	delay = SYSCTL_RCGCGPIO_R;
	//delay = SYSCTL_RCGCGPIO_R;
	delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;      // disable analog functionality on PF
}

void GPIO_Init(void){  
	volatile unsigned long delay;
	SYSCTL_RCGCGPIO_R |= 0x20;
  delay = SYSCTL_RCGCGPIO_R;
	//SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;    // activate port F
	//delay=SYSCTL_RCGC2_R; 									 // waiting for the hardware to setup
	GPIO_PORTF_DIR_R |= 0x04;                // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;             // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;                // enable digital I/O on PF2
                                           // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
	//GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F 
	//GPIO_PORTF_AMSEL_R &= ~0x04; //disable analog on PF2
	GPIO_PORTF_AMSEL_R = 0;
  GPIO_PORTF_DATA_R &= ~0x04;              // turn off LED
}
//******** Robot *************** 
// foreground thread, accepts data from producer
// inputs:  none
// outputs: none
void Robot(void){   
unsigned long data;      // ADC sample, 0 to 1023
unsigned long voltage;   // in mV,      0 to 3000
unsigned long time;      // in 10msec,  0 to 1000 
unsigned long t=0;
  OS_ClearMsTime();    
  DataLost = 0;          // new run with no lost data 
  printf("Robot running...");
  eFile_RedirectToFile("Robot");
  printf("time(sec)\tdata(volts)\n\r");
  do{
    t++;
    time=OS_MsTime();            // 10ms resolution in this OS
    data = OS_Fifo_Get();        // 1000 Hz sampling get from producer
    voltage = (300*data)/1024;   // in mV
    printf("%0u.%02u\t%0u.%03u\n\r",time/100,time%100,voltage/1000,voltage%1000);
  }
  while(time < 1000);       // change this to mean 10 seconds
  eFile_EndRedirectToFile();
  printf("done.\n\r");
  Running = 0;                // robot no longer running
  OS_Kill();
}
  
//************ButtonPush*************
// Called when Select Button pushed
// background threads execute once and return
void ButtonPush(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two robot threads
    NumCreated += OS_AddThread(&Robot,128,1);  // start a 20 second run
  }
}
//************DownPush*************
// Called when Down Button pushed
// background threads execute once and return
void DownPush(void){

}



//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 1 kHz, started by your ADC_Collect
// The timer triggers the ADC, creating the 1 kHz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 10-bit sample 
// sends data to the Robot, runs periodically at 1 kHz
// inputs:  none
// outputs: none
void Producer(unsigned long data){  
  if(Running){
    if(OS_Fifo_Put(data)){     // send to Robot
      NumSamples++;
    } else{ 
      DataLost++;
    } 
  }
}
 
//******** IdleTask  *************** 
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
unsigned long Idlecount=0;
void IdleTask(void){ 
  while(1) { 
    Idlecount++;        // debugging 
  }
}


//******** Interpreter **************
// your intepreter from Lab 4 
// foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
char cmd[8];
char fileName[16];
char p;
int kk;
void Interpreter(void) {
// add the following commands, remove commands that do not make sense anymore
// 1) format 
// 2) directory 
// 3) print file
// 4) delete file
// execute   eFile_Init();  after periodic interrupts have started

	while(1) {
		printf("Please Select.\r\n");
	  printf("1: format, 2: display directory, 3: display file, 4: delete file, 5: create file.\r\n");
		UART_InString(cmd, 8);
		printf("\r\n");
		if (strcmp(cmd,"1")==0){
			if(eFile_Format())
				printf("Formatting failure.\r\n");
			else 
				printf("Format done.\r\n");
		}
		else if (strcmp(cmd,"2")==0){
			eFile_Directory(&UART_OutChar);
		}
		else if (strcmp(cmd,"3")==0){
			printf("Type in filename:\r\n");
			UART_InString(fileName, 15);
			if (eFile_ROpen(fileName)){
				printf("Err: Interpreter cmd 3.\r\n"); break;
			}
			printf("\n");
			while (!eFile_ReadNext(&p)) {
				//eFile_ReadNext(&p);
				UART_OutChar(p);
			}
			/*for(kk=0;kk<1000;kk++){
				eFile_ReadNext(&p);
				UART_OutChar(p);
			}*/
			eFile_RClose();
		}
		else if (strcmp(cmd,"4")==0){ 
			printf("Type in filename:\r\n");
			UART_InString(fileName, 15);
			printf("\r\n");
			if (eFile_Delete(fileName)){
				printf("Err: Interpreter cmd 4.\r\n"); break;
			}
		}
		else if (strcmp(cmd,"5")==0){
			printf("Type in filename:\r\n");
			UART_InString(fileName, 15);
			printf("\r\n");
			if (strlen(fileName)>8) {
				printf("Invalid filename.\r\n"); break;
			}
			if (eFile_Create(fileName)){
				printf("Err: Interpreter cmd 5.\r\n"); break;
			}
		}
		else {
			printf("No such command.");
		}
	}
}


//*******************lab 5 main **********
int main(void){        // lab 5 real main
	int i;
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	GPIO_Init();
  UART_Init();
  Running = 0;         // robot not running
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
	if(eFile_Init()) 
		printf("Initial failure\r\n");
//********initialize communication channels
  OS_Fifo_Init(512);    // ***note*** 4 is not big enough*****
  ADC_Collect(0, 1000, &Producer); // start ADC sampling, channel 0, 1000 Hz

//*******attach background tasks***********
  OS_AddSW1Task(&ButtonPush,2);
  OS_AddSW2Task(&DownPush,3);

  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&IdleTask,128,7);  // runs when nothing useful to do
 
	eFile_Format();
	if (eFile_Create("Robot"))    printf("Error eFlie_Create\r\n");
	/*create a new file*/
	if(eFile_Create("file1"))     printf("Error eFile_Create\r\n");
  if(eFile_WOpen("file1"))      printf("Error eFile_WOpen\r\n");
  for(i=0;i<1000;i++){
    if(eFile_Write('a'+i%26))   printf("Error eFile_Write\r\n");
    if(i%52==51){
      if(eFile_Write('\n'))     printf("Error eFile_Write\r\n");  
      if(eFile_Write('\r'))     printf("Error eFile_Write\r\n");
    }
  }
  if(eFile_WClose())            printf("Error eFile_Close");
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//*****************test programs*************************
unsigned char buffer[512];
#define MAXBLOCKS 100
void diskError(char* errtype, unsigned long n){
  printf(errtype);
  printf(" disk error %u",n);
  OS_Kill();
}
void TestDisk(void){  DSTATUS result;  unsigned short block;  int i; unsigned long n;
  // simple test of eDisk
  printf("\n\rEE345M/EE380L, Lab 5 eDisk test\n\r");
  result = eDisk_Init(0);  // initialize disk
  if(result) diskError("eDisk_Init",result);
  printf("Writing blocks\n\r");
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;        
    }
    GPIO_PF3 = 0x08;     // PF3 high for 100 block writes
    if(eDisk_WriteBlock(buffer,block))diskError("eDisk_WriteBlock",block); // save to disk
    GPIO_PF3 = 0x00;     
  } 
  printf("Reading blocks\n\r");
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    GPIO_PF2 = 0x04;     // PF2 high for one block read
    if(eDisk_ReadBlock(buffer,block))diskError("eDisk_ReadBlock",block); // read from disk
    GPIO_PF2 = 0x00;
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer[i] != (0xFF&n)){
        printf("Read data not correct, block=%u, i=%u, expected %u, read %u\n\r",block,i,(0xFF&n),buffer[i]);
        OS_Kill();
      }      
    }
  }  
  printf("Successful test of %u blocks\n\r",MAXBLOCKS);
  OS_Kill();
}
void RunTest(void){
  NumCreated += OS_AddThread(&TestDisk,128,1);  
}
//******************* test main1 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
int testmain1(void){   // testmain1
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	GPIO_Init();
  UART_Init();

//*******attach background tasks***********
  //OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  OS_AddSW1Task(&RunTest,2);
  
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&TestDisk,128,1);  
  NumCreated += OS_AddThread(&IdleTask,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

void TestFile(void){   int i; char data; 
  printf("\n\rEE345M/EE380L, Lab 5 eFile test\n\r");
  // simple test of eFile
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Format())            diskError("eFile_Format",0); 
  eFile_Directory(&UART_OutChar);
  if(eFile_Create("file1"))     diskError("eFile_Create",0);
  if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
  for(i=0;i<1000;i++){
    if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
    if(i%52==51){
      if(eFile_Write('\n'))     diskError("eFile_Write",i);  
      if(eFile_Write('\r'))     diskError("eFile_Write",i);
    }
  }
  if(eFile_WClose())            diskError("eFile_Close",0);
  eFile_Directory(&UART_OutChar);
  if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
  for(i=0;i<1000;i++){
    if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",i);
    UART_OutChar(data);
  }
  if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
  eFile_Directory(&UART_OutChar);
  printf("Successful test of creating a file\n\r");
  OS_Kill();
}

//******************* test main2 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
int testmain2(void){ 
//int main(void){
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	GPIO_Init();
  UART_Init();
//*******attach background tasks***********
  //OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&TestFile,128,1);  
  NumCreated += OS_AddThread(&IdleTask,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}
