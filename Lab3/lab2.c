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
#include <string.h> 
//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);
Sema4Type LCDdisplay;
unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 400            // producer/consumer sampling
#define RUNLENGTH (20*FS) // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US // DAS 2kHz sampling period in system time units
long x[64],y[64];         // input and output arrays for FFT

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
//------------------Task 1--------------------------------
// 2 kHz sampling ADC channel 1, using software start trigger
// background thread executed at 2 kHz
// 60-Hz notch high-Q, IIR filter, assuming fs=2000 Hz
// y(n) = (256x(n) -503x(n-1) + 256x(n-2) + 498y(n-1)-251y(n-2))/256 (2k sampling)
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
long Filter(long data){
static long x[6]; // this MACQ needs twice
static long y[6];
static unsigned long n=3;   // 3, 4, or 5
  n++;
  if(n==6) n=3;     
  x[n] = x[n-3] = data;  // two copies of new data
  y[n] = (256*(x[n]+x[n-2])-503*x[n-1]+498*y[n-1]-251*y[n-2]+128)/256;
  y[n-3] = y[n];         // two copies of filter outputs too
  return y[n];
} 
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// samples channel 4, PD3,
// inputs:  none
// outputs: none
unsigned long DASoutput;
void DAS(void){ 
// Lab2
/*unsigned long input;  
unsigned static long LastTime;  // time at previous ADC sample
unsigned long thisTime;         // time at current ADC sample
long jitter = 0;                    // time between measured and expected, in us
  if(NumSamples < RUNLENGTH){   // finite time run
    //PE1 ^= 0x02;
		//GPIO_PORTF_DATA_R ^= 0x04; 
    input = ADC_In();           // channel set when calling ADC_Init
    //PE1 ^= 0x02;
		//GPIO_PORTF_DATA_R ^= 0x04; 
    thisTime = OS_Time();       // current time, 12.5 ns
		//printf("time = kkk\r\n");
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    if(FilterWork>1){    // ignore timing of first interrupt
      unsigned long diff = OS_TimeDifference(LastTime,thisTime);
      if(diff>PERIOD){
        jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
      }else{
        jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter){
        MaxJitter = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize){
        jitter = JITTERSIZE-1;
      }
      JitterHistogram[jitter]++; 
    }
    LastTime = thisTime;
    PE0 ^= 0x01;
		//GPIO_PORTF_DATA_R ^= 0x04; 
		*/
		unsigned long input;  
		
		if(NumSamples < RUNLENGTH){   // finite time run
    PE0 ^= 0x01;
	  input = ADC_In();           // channel set when calling ADC_Init
    PE0 ^= 0x01;
		OS_JitterSetThisTime(0);//thisTime = OS_Time();       // current time, 12.5 ns
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
			
    if(FilterWork>1){    // ignore timing of first interrupt
			OS_JitterCalc(0, PERIOD);
    }
		
    OS_JitterSetLastTime(0);//LastTime = thisTime;
    PE0 ^= 0x01;
  }
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
unsigned long myId = OS_Id(); 
  //PE1 ^= 0x02;
  ST7735_Message(1,0,"NumCreated =",NumCreated); 
  //PE1 ^= 0x02;
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,1,"PIDWork     =",PIDWork);
  ST7735_Message(1,2,"DataLost    =",DataLost);
	ST7735_Message(1,3,"Jitter 0.1us=",OS_MaxJitter(0));
  //PE1 ^= 0x02;
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
    if(OS_MsTime() > 20){ // debounce
	  //printf("time is %d\n",t);
      if(OS_AddThread(&ButtonWork,100,4)){
        NumCreated++; 
  		  //GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
			}
    //}
		//GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,1)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 400Hz, started by your ADC_Collect
// The timer triggers the ADC, creating the 400Hz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none

void Producer(unsigned long data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;
		PE1 ^= 0x2;
		// number of samples
    if(OS_Fifo_Put(data) == 0){ // send to consumer
      DataLost++;
    } 
  } 
}
void Display(void);
//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
unsigned short buffer2[100];
void Consumer(void){ 
unsigned long data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
unsigned long t;                  // time in 2.5 ms
unsigned long myId = OS_Id(); 
  ADC_Collect(5, FS, &Producer); // start ADC sampling, channel 5, PD2, 400 Hz
  NumCreated += OS_AddThread(&Display,128,0); 
  while(NumSamples < RUNLENGTH) { 
    //PE2 ^= 0x04;
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
			PE2 ^= 0x04;
      data = OS_Fifo_Get();    // get from producer
			PE2 ^= 	0x04;
			//printf("data is %d\n\r",data);
      x[t] = data;             // real part is 0 to 4095, imaginary part is 0
    }
    //PE2 ^= 0x04;
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
		//printf("send_data = %d\r\n",DCcomponent);
  }
  OS_Kill();  // done
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
unsigned long data,voltage;
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
	//ST7735_Message(0,1,"Run length = ",1);
  while(NumSamples < RUNLENGTH) { 
    data = OS_MailBox_Recv();
		//printf("data= %d\r\n",data);
    voltage = 3000*data/4095;               // calibrate your device so voltage is in mV
    PE3 ^= 0x08;
    ST7735_Message(0,2,"v(mV) =",voltage);  
    PE3 ^= 0x08;
  } 
  OS_Kill();  // done
} 

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
  while(NumSamples < RUNLENGTH) { 
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
  }
  for(;;){ }          // done
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
/*
void Interpreter(void){
	while (1){
		printf("Please Select.\r\n");
	  printf("1:performance measures,2:debugging parameters.\r\n");
		UART_InString(cmd, 8);
		printf("\n");
		if (strcmp(cmd,"1")==0){
			printf("NumSamples:%d\r\n", NumSamples);
			printf("NumCreated:%d\r\n", NumCreated);
			printf("DataLost:%d\r\n",DataLost);
			printf("FilterWork:%d\r\n",FilterWork);
			printf("PIDwork:%d\r\n",PIDWork);
			printf("MaxJitter:%d\r\n",OS_MaxJitter(0));
			printf("JitterHistogram:%d\r\n",OS_JitterHistogram(OS_MaxJitter(0), 0));
		}
		else if (strcmp(cmd,"2")==0){
			for(it = 0; it < 6; it++)
				printf("x[%d]:%d\r\n",it,x[it]);
			for(it = 0; it < 6; it++)
				printf("y[%d]:%d\r\n",it,y[it]);
		}
	}
  return;
} */   // just a prototype, link to your interpreter
void Interpreter(void){
	int cnt = 0;
	char tmp;
	while (1){
		printf("Please Select.\r\n");
	  printf("1:performance measures,2:debugging parameters,3:event timestamp,4: event reset.\r\n");
		UART_InString(cmd, 8);
		printf("\n");
		if (strcmp(cmd,"1")==0){
			printf("NumSamples:%d\r\n", NumSamples);
			printf("NumCreated:%d\r\n", NumCreated);
			printf("DataLost:%d\r\n",DataLost);
			printf("FilterWork:%d\r\n",FilterWork);
			printf("PIDwork:%d\r\n",PIDWork);
			printf("MaxJitter:%d\r\n",OS_MaxJitter(0));
			printf("JitterHistogram:%d\r\n",OS_JitterHistogram(OS_MaxJitter(0), 0));
		}
		else if (strcmp(cmd,"2")==0){
			for(it = 0; it < 6; it++)
				printf("x[%d]:%d\r\n",it,x[it]);
			for(it = 0; it < 6; it++)
				printf("y[%d]:%d\r\n",it,y[it]);
		}
		else if (strcmp(cmd,"3")==0){
			while(cnt < eventNum) {
				tmp = event[cnt];
				//printf("tmp is %c\r\n",tmp);
				if (tmp=='1') 
					printf("Event %d: A foreground thread is started at time: %lu\r\n", cnt, timeStamp[cnt]);
				else if (tmp=='2')
					printf("Event %d: A periodic thread is started at time: %lu\r\n", cnt, timeStamp[cnt]);
				else if (tmp=='3')
					printf("Event %d: A periodic thread finishes at time: %lu\r\n", cnt, timeStamp[cnt]);

				cnt++;
			}
			//printf("Event number:%d\r\n",eventNum);
			cnt = 0;
			printf("Max disable interrupt time: %lu\r\n",max_interrupt);
			//printf("Percentage disable interrupt: %f\r\n",total_interrupt/OS_Time());
		}
		else if (strcmp(cmd,"4")==0){
			eventNum = 0;
			max_interrupt = 0;
		}
	}
  return;
}    // just a prototype, link to your interpreter

// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 
//--------------end of Task 5-----------------------------
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);
//*******************final user main DEMONTRATE THIS TO TA**********
int main0(void) {
//int main(void){ 
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	GPIO_Init();
  UART_Init();
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
  //MaxJitter = 0;       // in 1us units

//********initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(4);    // ***note*** 4 is not big enough*****
	OS_InitSemaphore(&LCDdisplay, 1);
	//ST7735_InitR(INITR_REDTAB);


//*******attach background tasks***********
    OS_AddSW1Task(&SW1Push,2);
    OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3
    ADC_Init(4);  // sequencer 2, channel 4, PD3, sampling in DAS() 
    OS_AddPeriodicThread(&DAS,PERIOD,1); // 2 kHz real time sampling of PD3
	  //OS_MsTime();
    NumCreated = 0 ;
	
// create initial foreground threads
	
    
    NumCreated += OS_AddThread(&Consumer,128,1); 
    NumCreated += OS_AddThread(&PID,128,5);  // Lab 3, make this lowest priority
		NumCreated += OS_AddThread(&Interpreter,128,2); 
    OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 
//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 1 part 1)
// run this with 
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops
void Thread1(void){
	
  Count1 = 0;          
  for(;;){
    PE1 ^= 0x02;       // heartbeat
    Count1++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread2(void){
	
  Count2 = 0;          
  for(;;){
    PE2 ^= 0x04;       // heartbeat
    Count2++;
    OS_Suspend();      // cooperative multitasking
  }
}
void Thread3(void){
	
  Count3 = 0;          
  for(;;){
    PE3 ^= 0x08;       // heartbeat
    Count3++;
    OS_Suspend();      // cooperative multitasking
  }
}

//Testmain1
//int main(void){
int Testmain1(void){  // Testmain1
  OS_Init();          // initialize, disable interrupts
  PortE_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(800000); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 1 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
Sema4Type testmutex;
void Thread1b(void){
	long k;
  Count1 = 0;          
  //for(k=0;k<500000;k++){
	for(;;){
		//printf("1\r\n");
    PE1 ^= 0x02;       // heartbeat
		//GPIO_PORTF_DATA_R ^= 0x04;
		//OS_Sleep(4);
		OS_bWait(&testmutex);
    Count1++;
  }
	//OS_Sleep(160);
	/*for(;;){
		Count1++;
		PE1 ^= 0x02;
	}*/
	//OS_Kill();
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
		//printf("22\r\n");
    PE2 ^= 0x04;       // heartbeat
		OS_bSignal(&testmutex);
		//OS_Sleep(4);
		//OS_bWait(&testmutex);
    Count2++;
  }
}
void Thread3b(void){
  Count3 = 0;          
  for(;;){
		//printf("333\r\n");
    PE3 ^= 0x08;       // heartbeat
    Count3++; 
		//OS_bSignal(&testmutex);
  }
}
int main(void){
//int Testmain2(void){  // Testmain2
  OS_Init();           // initialize, disable interrupts
  PortE_Init();       // prof ile user threads
	GPIO_Init();
	OS_InitSemaphore(&testmutex, 0);
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1); 
  NumCreated += OS_AddThread(&Thread2b,128,2); 
  NumCreated += OS_AddThread(&Thread3b,128,4); 
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
	//OS_Launch(1000000);
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 1 part 2)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
  Count1++;
	//GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
	//printf("b1b1b1b1b1b1b1\r\n");
  OS_Signal(&Readyc);
}
void Thread5c(void){
	int num;
  for(;;){
		num = NumCreated;
		//printf("5\r\n");
    OS_Wait(&Readyc);
    Count5++;   // Count2 + Count5 should equal Count1 
    Lost = Count1-Count5-Count2;
  }
}
void Thread2c(void){
	//GPIO_PORTF_DATA_R ^= 0x04;   
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called      
  Count2 = 0;    
  Count5 = 0;    // Count2 + Count5 should equal Count1  
  NumCreated += OS_AddThread(&Thread5c,128,3); 
  OS_AddPeriodicThread(&BackgroundThread1c,TIME_1MS,0); 
	//OS_AddPeriodicThread(&BackgroundThread1c,10000000,0);
  for(;;){
		//printf("2\r\n");
    OS_Wait(&Readyc);
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;          
  for(;;){
		//printf("3\r\n");
    Count3++;
  }
}
void Thread4c(void){	
	int i;
	printf("4444\t\n");
  for(i=0;i<64;i++){
    Count4++;
    OS_Sleep(10);
		//OS_Suspend();
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
	GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
  NumCreated += OS_AddThread(&Thread4c,128,3); 
}
//int main(void){  
int Testmain3(void){   // Testmain3
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	UART_Init();
	GPIO_Init();
	//UART_Init();
// Count2 + Count5 should equal Count1
  NumCreated = 0 ;
	//OS_AddPeriodicThread(&BackgroundThread1c,80000000,0); 
  OS_AddSW1Task(&BackgroundThread5c,2);
  NumCreated += OS_AddThread(&Thread2c,128,2); 
  NumCreated += OS_AddThread(&Thread3c,128,3); 
  NumCreated += OS_AddThread(&Thread4c,128,3); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, run this example (Lab 1 part 2)
// Count1 should exactly equal Count2
// Count3 should be very large
// Count4 increases by 640 every time select is pressed
// NumCreated increase by 1 every time select is pressed

// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
void BackgroundThread1d(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readyd);
		//GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;          
  Count2 = 0;          
  for(;;){
    OS_bWait(&Readyd);
    Count2++;     
  }
}
void Thread3d(void){
  Count3 = 0;          
  for(;;){
    Count3++;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<640;i++){
    Count4++;
    OS_Sleep(1);
  }
  OS_Kill(); 
}
void BackgroundThread5d(void){   // called when Select button pushed
	GPIO_PORTF_DATA_R ^= 0x04;              // turn off LED
  NumCreated += OS_AddThread(&Thread4d,128,3); 
}
//int main(void){
int Testmain4(void){   // Testmain4
  Count4 = 0;          
  OS_Init();           // initialize, disable interrupts
	GPIO_Init();
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1d,PERIOD,0); 
  OS_AddSW1Task(&BackgroundThread5d,2);
  NumCreated += OS_AddThread(&Thread2d,128,2); 
  NumCreated += OS_AddThread(&Thread3d,128,3); 
  NumCreated += OS_AddThread(&Thread4d,128,3); 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//******************* Lab 3 Preparation 2**********
// Modify this so it runs with your RTOS (i.e., fix the time units to match your OS)
// run this with 
// UART0, 115200 baud rate, used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts
// SW2 no interrupts
unsigned long CountA;   // number of times Task A called
unsigned long CountB;   // number of times Task B called
unsigned long Count1;   // number of times thread1 loops


//*******PseudoWork*************
// simple time delay, simulates user program doing real work
// Input: amount of work in 100ns units (free free to change units
// Output: none
void PseudoWork(unsigned short work){
unsigned short startTime;
  startTime = OS_Time();    // time in 100ns units
  while(OS_TimeDifference(startTime,OS_Time()) <= work){} 
}
void Thread6(void){  // foreground thread
  Count1 = 0;          
  for(;;){
    Count1++; 
    PE0 ^= 0x01;        // debugging toggle bit 0  
  }
}

extern void Jitter(void){
	int k;
	int end;
	UART_OutString("MaxJitter1:");
	UART_OutUDec(OS_MaxJitter(1));
	printf("\r\n");
	UART_OutString("Histogram1:");
	printf("\r\n");
	if(OS_MaxJitter(1)>64)
		end = 63;
	else
		end = OS_MaxJitter(1);
	for(k = 0; k <= end; k++){
		printf("jitter %d:",k);
		UART_OutUDec(OS_JitterHistogram(k, 1));
		printf("\r\n");
	}
	//printf("\r\n");
	UART_OutString("MaxJitter2:");
	UART_OutUDec(OS_MaxJitter(2));
	printf("\r\n");
	
	UART_OutString("Histogram2:");
	//
	printf("\r\n");
	if(OS_MaxJitter(2)>64)
		end = 63;
	else
		end = OS_MaxJitter(2);
	for(k = 0; k <= end;k++){
		printf("jitter %d:",k);
		UART_OutUDec(OS_JitterHistogram(k, 2));
		printf("\r\n");
	}
	printf("Max disable interrupt time: %lu\r\n",max_interrupt);
	//printf("total interrupt time: %lu\r\n",total_interrupt);
	return;
	
}   // prints jitter information (write this)
void Thread7(void){  // foreground thread
  UART_OutString("\n\rEE345M/EE380L, Lab 3 Preparation 2\n\r");
  OS_Sleep(5000);   // 10 seconds        
  Jitter();         // print jitter information
  UART_OutString("\n\r\n\r");
  OS_Kill();
}
#define workA 500       // {5,50,500 us} work in Task A
#define counts1us 80    // number of OS_Time counts per 1us
#define PERIODA TIME_1MS*2.99
#define PERIODB TIME_1MS*2
//unsigned long jitterCount1 = 0;
//unsigned long jitterCount2 = 0;

void TaskA(void){       // called every {1000, 2990us} in background
  unsigned long input;
	PE1 = 0x02;      // debugging profile
  //input = ADC_In();  
	OS_JitterSetThisTime(1);
	CountA++;
	if (CountA>1){
		OS_JitterCalc(1,PERIODA);
	}
	OS_JitterSetLastTime(1);
  PseudoWork(workA*counts1us); //  do work (100ns time resolution)
  PE1 = 0x00;      // debugging profile  
}
#define workB 250       // 250 us work in Task B
void TaskB(void){       // called every pB in background
  PE2 = 0x04;      // debugging profile  
	OS_JitterSetThisTime(2);
	CountB++;
	if (CountB>1) {
		OS_JitterCalc(2, PERIODB);
	}
	OS_JitterSetLastTime(2);
  PseudoWork(workB*counts1us); //  do work (100ns time resolution)
	PE2 = 0x00;      // debugging profile  
}

//int main(void){
int Testmain5(void){       // Testmain5 Lab 3
  PortE_Init();
  OS_Init();           // initialize, disable interrupts
	UART_Init();
	//ADC_Init(4);
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread6,128,2); 
  NumCreated += OS_AddThread(&Thread7,128,1); 
  OS_AddPeriodicThread(&TaskA,TIME_1MS*2.99,0);           // 1 ms, higher priority
  OS_AddPeriodicThread2(&TaskB,TIME_1MS*2,1);         // 2 ms, lower priority
 
  OS_Launch(TIME_2MS); // 2ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Preparation 4**********
// Modify this so it runs with your RTOS used to test blocking semaphores
// run this with 
// UART0, 115200 baud rate,  used to output results 
// SYSTICK interrupts, period established by OS_Launch
// first timer interrupts, period established by first call to OS_AddPeriodicThread
// second timer interrupts, period established by second call to OS_AddPeriodicThread
// SW1 no interrupts, 
// SW2 no interrupts
Sema4Type s;            // test of this counting semaphore
unsigned long SignalCount1;   // number of times s is signaled
unsigned long SignalCount2;   // number of times s is signaled
unsigned long SignalCount3;   // number of times s is signaled
unsigned long WaitCount1;     // number of times s is successfully waited on
unsigned long WaitCount2;     // number of times s is successfully waited on
unsigned long WaitCount3;     // number of times s is successfully waited on

#define MAXCOUNT 20000
//#define MAXCOUNT 20
void OutputThread(void){  // foreground thread
  UART_OutString("\n\rEE345M/EE380L, Lab 3 Preparation 4\n\r");
  while(SignalCount1+SignalCount2+SignalCount3<100*MAXCOUNT){
    OS_Sleep(1000);   // 1 second
    UART_OutString(".");
  }       
  UART_OutString(" done\n\r");
  UART_OutString("Signalled="); UART_OutUDec(SignalCount1+SignalCount2+SignalCount3);
  UART_OutString(", Waited="); UART_OutUDec(WaitCount1+WaitCount2+WaitCount3);
  UART_OutString("\n\r");
  OS_Kill();
}
void Wait1(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount1++; 
  }
}
void Wait2(void){  // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount2++; 
  }
}
void Wait3(void){   // foreground thread
  for(;;){
    OS_Wait(&s);    // three threads waiting
    WaitCount3++; 
  }
}
void Signal1(void){      // called every 799us in background
  if(SignalCount1<MAXCOUNT){
    OS_Signal(&s);
    SignalCount1++;
  }
}
// edit this so it changes the periodic rate
void Signal2(void){       // called every 1111us in background
  if(SignalCount2<MAXCOUNT){
    OS_Signal(&s);
    SignalCount2++;
  }
}
void Signal3(void){       // foreground
  while(SignalCount3<98*MAXCOUNT){
    OS_Signal(&s);
    SignalCount3++;
  }
  OS_Kill();
}

long add(const long n, const long m){
static long result;
  result = m+n;
  return result;
}

//int main(void){
int Testmain6(void){      // Testmain6  Lab 3
  volatile unsigned long delay;
  OS_Init();           // initialize, disable interrupts
  delay = add(3,4);
  PortE_Init();
	UART_Init();
  SignalCount1 = 0;   // number of times s is signaled
  SignalCount2 = 0;   // number of times s is signaled
  SignalCount3 = 0;   // number of times s is signaled
  WaitCount1 = 0;     // number of times s is successfully waited on
  WaitCount2 = 0;     // number of times s is successfully waited on
  WaitCount3 = 0;	  // number of times s is successfully waited on
  OS_InitSemaphore(&s,0);	 // this is the test semaphore
  OS_AddPeriodicThread(&Signal1,(799*TIME_1MS)/1000,0);   // 0.799 ms, higher priority
  OS_AddPeriodicThread2(&Signal2,(1111*TIME_1MS)/1000,1);  // 1.111 ms, lower priority
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread6,128,6);    	// idle thread to keep from crashing
  NumCreated += OS_AddThread(&OutputThread,128,2); 	// results output thread
  NumCreated += OS_AddThread(&Signal3,128,2); 	// signalling thread
  NumCreated += OS_AddThread(&Wait1,128,2); 	// waiting thread
  NumCreated += OS_AddThread(&Wait2,128,2); 	// waiting thread
  NumCreated += OS_AddThread(&Wait3,128,2); 	// waiting thread
 
  OS_Launch(TIME_1MS);  // 1ms, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//******************* Lab 3 Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed 
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed, 
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PE0 to measure context switch time
void Thread8(void){       // only thread running
  while(1){
    PE1 ^= 0x02;      // debugging profile  
  }
}
//int main(void){
int Testmain7(void){       // Testmain7
  PortE_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread8,128,2); 
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

unsigned short buffer[100];
int main3(void){
  char string[20];  // global to assist in debugging
  PLL_Init();               // set system clock to 50 MHz
	GPIO_Init();
	ST7735_InitR(INITR_REDTAB); // initialize LCD
	ST7735_FillScreen(0xFFFF); // set screen to white
	PortE_Init();
  UART_Init();              // initialize UART
  while(1){
    /*printf("Pleast input the command\r\n");
    UART_InString(string,19);
    OutCRLF();
		interpreter (string);*/
//		ADC_Collect(0,1000,buffer,1);
	//	  while(ADC_Status()){};
			//printf("-----ADC Result -----\r\n");
		//ST7735_InitR(INITR_REDTAB); // initialize LCD
	  //ST7735_FillScreen(0xFFFF); // set screen to white
		ST7735_Message(0,1,"Run length = ",1);   // top half used for Display
			
  }	
}

