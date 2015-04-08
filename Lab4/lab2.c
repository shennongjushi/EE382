// Lab2.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3 Testmain4  and main
// Lab3: Testmain5 Testmain6, Testmain7, and main (with SW2)

// Jonathan W. Valvano 1/31/14, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task, samples PD3
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PD3 Ain3 sampled at 2k, sequencer 3, by DAS software start in ISR
// PD2 Ain5 sampled at 250Hz, sequencer 0, by Producer, timer tigger

#include "OS.h"
#include "inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "ADC_SW.h"
#include "UART.h"
#include "PLL.h"
//#include <math.h>
#include <string.h> 

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
void cr4_fft_1024_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);
//extern Sema4Type LCDdisplay;
unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
//#define FS 400            // producer/consumer sampling
#define RUNLENGTH (20*FS) // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US // DAS 2kHz sampling period in system time units
long x[64],y[64];         // input and output arrays for FFT

//------------------Lab 4---------------------------------
#define FS 2000 //Timer trigger ADC sampling rate
#define PERIOD_ADC 1000*TIME_1MS/FS // SW trigger ADC sampling rate
#define NumDiscreteSamples	64
long beforeFIR[NumDiscreteSamples] = {0, };
long afterFIR[NumDiscreteSamples] = {0, };
long afterFFT[NumDiscreteSamples] = {0, };
long afterFFT_Raw[NumDiscreteSamples] = {0,};
volatile int graphFlag = 0;
// Newton's method
// s is an integer
// sqrt(s) is an integer
unsigned long sqrt_my(unsigned long s){
unsigned long t;         // t*t will become s
int n;                   // loop counter to make sure it stops running
  t = s/10+1;            // initial guess 
  for(n = 16; n; --n){   // guaranteed to finish
    t = ((t*t+s)/t)/2;  
  }
  return t; 
}
//------------------End of Lab 4--------------------------

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
// Lab2
/*long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,}; */
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


//------------------Lab 4---------------------------------
// FIR filter
//long h[51]={4,-1,-8,-14,-16,-10,-1,6,5,-3,-13,
//     -15,-8,3,5,-5,-20,-25,-8,25,46,26,-49,-159,-257,
//     984,-257,-159,-49,26,46,25,-8,-25,-20,-5,5,3,-8,
//     -15,-13,-3,5,6,-1,-10,-16,-14,-8,-1,4};
/*long h[51]={0,0,0,0,0,0,0,0,0,1,0,
     -2,0,3,-1,-4,2,6,-6,-8,11,9,-24,-10,80,
     139,80,-10,-24,9,11,-8,-6,6,2,-4,-1,3,0,
     -2,0,1,0,0,0,0,0,0,0,0,0};*/
/*long h[51]={0,0,0,0,0,0,-1,1,-1,0,0,
     -1,1,-1,1,-1,2,-1,1,-1,3,-9,19,-31,40,
     213,40,-31,19,-9,3,-1,1,-1,2,-1,1,-1,1,
     -1,0,0,-1,1,-1,0,0,0,0,0,0};
*/
const long h[51]={0,0,1,0,-1,1,1,-1,0,1,-1,
     0,0,0,1,-1,-3,4,3,-10,2,16,-17,-21,78,
     151,78,-21,-17,16,2,-10,3,4,-3,-1,1,0,0,
     0,-1,1,0,-1,1,1,-1,0,1,0,0};



		 
long Data[102]={0,};
long *Pt = &Data[0];
/*void Filter_Init(void) {
	Pt = &Data[0];
}	*/


//calculate one filter output
//called at sampling rate
//Input: new ADC data
//Output: filter output, DAC data
unsigned long filt_index = 50;
short Filter_Calc(long newdata) {
	long sr;
	int i;
	long sum;
	long *pt, *apt;
	/*
	sr = StartCritical();
	if (Pt == &Data[0]) {
			Pt = &Data[50]; // Wrap
	}
	else {
		Pt--;
	}
	*Pt = *(Pt + 51) = newdata;
	pt = Pt;
	apt = h;
	sum = 0;
	for(i = 51; i; i--) {
		sum += (*pt)*(*apt);
		apt++;
		pt++;
	}
	EndCritical(sr);
	return sum/256;
	*/
  sr = StartCritical();
  filt_index++;
  if(filt_index==102) filt_index = 51;

  Data[filt_index]=newdata;
	Data[filt_index-51]=newdata;

  sum=0;

  for(i=0;i<51;i++){
    sum = sum + h[i]*Data[filt_index-i];
  }

  sum = sum/256;
	EndCritical(sr);
	return sum;
}	


//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
unsigned long myId = OS_Id(); 	
	// Lab4
	if (graphFlag == 0)
		graphFlag = 1;
	else
		graphFlag = 0;
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
    if(OS_MsTime() > 20){ // debounce
			// Lab4
			if (graphFlag == 0)
				graphFlag = 1;
			else
				graphFlag = 0;
			OS_ClearMsTime();  // at least 20ms between touches
		}
}



//------------------Lab 4---------------------------------
#define SWtrigger 0
#define timeGraph 0
#define freqGraph 1
int graphType = -1;
int triggerMode = -1;
void Display_ADC(void);

void ADC_Timer_Producer(unsigned long data){  
	// number of samples
		//UART_OutUDec(data);
			//printf("\r\n");
   if(OS_Fifo_Put(data) == 0){ // send to consumer
     DataLost++;
   } 
}

void ADC_SW_Producer(void) {
	unsigned long data = ADC_In();
	if(OS_Fifo_Put(data) == 0){ // send to consumer
     DataLost++;
   } 
}

unsigned long idx;
volatile long data_collect;
volatile long lastdata_collect;
void ADC_Consumer(void) {
	unsigned long myId = OS_Id();
	
	while (triggerMode == -1) {}
	if (triggerMode == SWtrigger) { // SW trigger
		ADC_Init(4);
		OS_AddPeriodicThread(&ADC_SW_Producer, PERIOD_ADC, 4);
	}
	else { // Timer trigger
		ADC_Collect(5, FS, &ADC_Timer_Producer);
	}
	NumCreated += OS_AddThread(&Display_ADC,128,2);
	/*while (1) {
		for (idx = 0; idx < NumDiscreteSamples; idx++) {
			data_collect = OS_Fifo_Get();    // get from producer
			beforeFIR[idx] = data_collect;
			afterFIR[idx] = Filter_Calc(data_collect);
			if (graphType==timeGraph) OS_MailBox_Send(afterFIR[idx]);
		}
		cr4_fft_64_stm32(afterFFT,afterFIR,64);
		if (graphType==freqGraph) OS_MailBox_Send(0);
		}*/
	while (1) {
		for (idx = 0; idx < NumDiscreteSamples; idx++) {
			data_collect = OS_Fifo_Get();    // get from producer
			//UART_OutUDec(data_collect);
			//printf("\r\n");
			if(data_collect > 0){
				beforeFIR[idx] = data_collect;
			  afterFIR[idx] = Filter_Calc(data_collect);
			}
			else{
				beforeFIR[idx] = lastdata_collect;
			  afterFIR[idx] = Filter_Calc(lastdata_collect);
			}
			lastdata_collect = data_collect;
			
			if (graphType==timeGraph) OS_MailBox_Send(afterFIR[idx]);
		}
		cr4_fft_64_stm32(afterFFT,afterFIR,64);
		cr4_fft_64_stm32(afterFFT_Raw,beforeFIR,64);
		if (graphType==freqGraph) OS_MailBox_Send(0);
		}
//////////////////////
		 
  }


volatile int col=0, lastRow[128]={0,};
volatile int row = 0;
volatile int thread_count = 0;
volatile long data_display;
void Display_ADC(void) {
	long voltage, mag, re, im;
	int count;
	int i;
  while(graphType == -1) {}
	while (1) {
		thread_count++;
		if (graphFlag == 0) {
			if (graphType == timeGraph) {
				data_display = OS_MailBox_Recv();
				row=159-(data_display*159)/4096;
				col=(col+2)%128;
				//DisableInterrupts();
				ST7735_DrawPixel(col, lastRow[col], 0);
				/*for(i=0;i<160;i++){
					ST7735_DrawPixel(col, i, 0);
				}*/
				ST7735_DrawPixel(col, row, 65535);
				//EnableInterrupts();
				lastRow[col]=row;
			}
			else if (graphType == freqGraph) {
				OS_MailBox_Recv();
				for (count = 0; count < NumDiscreteSamples / 2; count++) {
					re = afterFFT[count] & 0xffff;
					im = afterFFT[count] >> 16;
					mag = sqrt_my(re*re + im*im);
					row=159-(mag*159)/3000;
					col=count*4;
					ST7735_DrawFastVLine(col, lastRow[col],160-lastRow[col], 0);
					ST7735_DrawFastVLine(col, row, 160-row, 1631);
					lastRow[col]=row;
				}
			}
		}
		else if (graphFlag == 1) {	// Stop graphing
			data_display = OS_MailBox_Recv();
		}
	}		
}


//------------------End of Lab 4--------------------------

 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
// foreground thread that runs without waiting or sleeping
// it executes a digital controller 
//******** PID *************** 
// foreground thread, runs a PID controller
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3];    // PID coefficients
short Actuator;
void PID(void){ 
short err;  // speed error, range -100 to 100 RPM
unsigned long myId = OS_Id(); 
  PIDWork = 0;
  IntTerm = 0;
  PrevError = 0;
  Coeff[0] = 384;   // 1.5 = 384/256 proportional coefficient
  Coeff[1] = 128;   // 0.5 = 128/256 integral coefficient
  Coeff[2] = 64;    // 0.25 = 64/256 derivative coefficient*
  /*while(NumSamples < RUNLENGTH) { 
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
		
  }*/
	//ST7735_DrawPixel(50, 150, 0);
  for(;;){ 
		//ST7735_DrawPixel(50, 150, 0);
	}          // done
}
//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
// UART background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
char cmd[8]; // global to assist in debugging
int m = 10000000;
int it;

//********* Measurement Data ***********//

// Lab3 Procedure 7
extern unsigned long eventNum;
extern char event[100];
extern unsigned long timeStamp[100];
extern unsigned long max_interrupt;
extern unsigned long total_interrupt;


//------------------Lab 4---------------------------------
void Interpreter(void){
	while (1){
		printf("Please Select Graph Mode.\r\n");
	  printf("0:Voltage v.s. Time, 1:Voltage v.s. Frequency.\r\n");
		UART_InString(cmd, 8);
		printf("\n");
		if (strcmp(cmd,"0")==0){
			graphType = 0;
			break;
		}
		else if (strcmp(cmd,"1")==0){
			graphType = 1;
			break;
		}
		else {
			printf("Invalid Graph Mode!\r\n");
		}
	}
	
	while (1){
		printf("Please Select ADC Trigger Mode.\r\n");
	  printf("0:SW Trigger, 1:Timer Trigger.\r\n");
		UART_InString(cmd, 8);
		printf("\n");
		if (strcmp(cmd,"0")==0){
			triggerMode = 0;
			printf("\r\n");
			break;
		}
		else if (strcmp(cmd,"1")==0){
			triggerMode = 1;
			printf("\r\n");
			break;
		}
		else {
			printf("Invalid Graph Mode!\r\n");
		}
	}
	
 OS_Kill();
	//while(1){}
}   
//------------------End of Lab 4---------------------------------

//--------------end of Task 5-----------------------------
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);
//*******************final user main DEMONTRATE THIS TO TA**********
int main(void) {
//int main(void){ 
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	GPIO_Init();
  UART_Init();
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  //MaxJitter = 0;       // in 1us units
	//graphType = 0;
	//triggerMode = 1;

//********initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(128);    // ***note*** 4 is not big enough*****
	//OS_InitSemaphore(&LCDdisplay, 1);


//*******attach background tasks***********
    OS_AddSW1Task(&SW1Push,2);
    //OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3
    //ADC_Init(4);  // sequencer 2, channel 4, PD3, sampling in DAS() 
    //OS_AddPeriodicThread(&DAS,PERIOD,1); // 2 kHz real time sampling of PD3
	  //OS_MsTime();
		//ADC_Init(4);
		//OS_AddPeriodicThread(&ADC_SW_Producer, PERIOD_ADC, 4);
    NumCreated = 0 ;
	
// create initial foreground threads
	
    
    NumCreated += OS_AddThread(&ADC_Consumer,128,3); 
    NumCreated += OS_AddThread(&PID,128,3);  // Lab 3, make this lowest priority
		NumCreated += OS_AddThread(&Interpreter,128,1); 
    OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

