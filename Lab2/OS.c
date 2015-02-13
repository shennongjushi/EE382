#include "inc/tm4c123gh6pm.h"
#include "OS.h"
#include "UART.h"
#include "FIFO.h"
#include "PLL.h"
#include "ST7735.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
void StartOS(void);
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void); // user function

#define FIFOSIZE 128
#define NUMTHREADS  10        // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack
long static Fifo[FIFOSIZE];
long volatile *PutPt;
long volatile *GetPt;
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
	int id;	 // a unique identifier for the thread, used for debugging
	unsigned int sleep; // used to suspend execution
	unsigned int priority;  //used in Lab3
	unsigned int blocked;   //used in Lab3
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
char valid[NUMTHREADS];
tcbType *RunPt;
//tcbType *head = NULL;
tcbType *tail = NULL;
int32_t Stacks[NUMTHREADS][STACKSIZE];
int j;
Sema4Type DataRoomLeft,  mutex, DataAvailable;//for background <==> foreground communication between fifo
Sema4Type DataValid, BoxFree;
unsigned long MailBox;
//unsigned int thread_number;


int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority) {
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  PeriodicTask = task;          // user function
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = period - 1;    // 4) reload value
	//TIMER1_TAPR_R = 0x00000000;
  TIMER1_ICR_R = 0x00000001;    // 5) clear TIMER1A timeout flag
  TIMER1_IMR_R = 0x00000001;    // 6) arm timeout interrupt
	
	NVIC_PRI5_R = (NVIC_PRI4_R&0xFFFF00FF)|(priority<<13); // 7) priority
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 21
  NVIC_EN0_R = 1 << 21;           // 8) enable IRQ 21 in NVIC
	
  TIMER1_CTL_R = 0x00000001;    // 9) enable TIMER1A
  EndCritical(sr);
	return 1;
}

void Timer1A_Handler(void) {
	TIMER1_ICR_R = 0x00000001;	// clear TIMER1A timeout flag
	(*PeriodicTask)(); 					// execute user function
}



// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  PLL_Init();                 // set processor clock to 80 MHz
	//UART_Init();                 // initialize uart
	ST7735_InitR(INITR_REDTAB); // initialize LCD
	ST7735_FillScreen(0xFFFF); // set screen to white
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // systick priority 7
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0xFF1FFFFF)|(7<<21); // pendsv priority 7
	for(j = 0; j< NUMTHREADS; j++){
		valid[j] = 0;
		tcbs[j].id = -1;
	}
	//thread_number = 0;
	RunPt = &tcbs[0];       // thread head will run first
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority)
{ int32_t status;
  status = StartCritical();
	for(j = 0; j < NUMTHREADS; j++){//find the position for the new task
		if(valid[j] == 0){
			valid[j] = 1;
			break;
		}
	}
	if(j == NUMTHREADS) return 0;// if there is no space available, return
	
	tcbs[j].blocked = 0;
	tcbs[j].id = j;
	tcbs[j].sleep = 0;
	tcbs[j].priority = priority;
	//thread_number++;
	/*if(head == NULL){
		head = &tcbs[j];
		head->next = head;
	}else{
		tcbs[j].next = head->next;
		head->next = &tcbs[j];
	}*/
	if(tail == NULL){
		tail = &tcbs[j];
		tail->next = tail;
	}else{
		tcbs[j].next = tail->next;
		tail->next = &tcbs[j];
		tail = &tcbs[j];
	}
  SetInitialStack(j); 
	Stacks[j][STACKSIZE-2] = (int32_t)(task);
  EndCritical(status);
  return 1;               // successful
}

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice){
	 NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
   NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
   StartOS();                   // start on the first task
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
	NVIC_ST_CURRENT_R = 0;
	NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PENDSTSET;
}


// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
	semaPt->Value = value;
} 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
		OS_DisableInterrupts();
	  while(semaPt->Value <= 0){
			OS_EnableInterrupts();
			OS_DisableInterrupts();
		}
		(semaPt->Value)--;
		OS_EnableInterrupts();	
}

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
	long status;
	status = StartCritical();
	semaPt->Value++;
	EndCritical(status);
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
	OS_DisableInterrupts();
	while(semaPt->Value == 0){
		OS_EnableInterrupts();
		OS_DisableInterrupts();
	}
	(semaPt->Value) = 0;
	OS_EnableInterrupts();
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
	long status;
	status = StartCritical();
	semaPt->Value = 1;
	EndCritical(status);
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void){
	return RunPt->id;
}



//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
#define PF4 (*((volatile unsigned long *)0x40025040))
unsigned long static LastPF4;
void (*PF4Task)(void);
int OS_AddSW1Task(void(*task)(void), unsigned long priority){
	volatile unsigned long delay;
	//SYSCTL_RCGC2_R |= 0x00000020;//activate clock for Port F
	//delay = SYSCTL_RCGC2_R; 		 //allow time for clock to start
	//delay = SYSCTL_RCGC2_R; 		 //allow time for clock to start
	//delay = SYSCTL_RCGC2_R; 		 //allow time for clock to start
  GPIO_PORTF_AMSEL_R &= ~0x10; //disable analog on PF1
	GPIO_PORTF_PCTL_R &= ~0x000F0000; //PCTL GPIO on PF4
	GPIO_PORTF_DIR_R &= ~0x10;  // PF4 is an input
	GPIO_PORTF_AFSEL_R &=~0x10; //regular port function
	GPIO_PORTF_PUR_R |= 0x10; //pull-up on PF4
	GPIO_PORTF_DEN_R |= 0x10; //enable difital I/O on PF4
	GPIO_PORTF_IS_R &= ~0x10; //PF4 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x10; //PF4 is not both edges
	GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
	GPIO_PORTF_ICR_R = 0x10; //clear flag4
	GPIO_PORTF_IM_R |= 0x10; //enable interrupt on PF4
	LastPF4 = PF4;
	NVIC_PRI7_R = (NVIC_PRI7_R&0xff00ffff)|(priority<<21);//priority 5
	NVIC_EN0_R = NVIC_EN0_INT30;//ENABLE INTERRUPT 30
	PF4Task = task;
	return 1;
}
void static DebounceTaskPF4(void){
	OS_Sleep(5);
	LastPF4 = PF4;
	GPIO_PORTF_ICR_R = 0x10; //clear flag4
	GPIO_PORTF_IM_R |= 0x10; //enable interrupt on PF4
	OS_Kill();
}


//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
#define PF0 (*((volatile unsigned long *)0x40025000))
unsigned long static LastPF0;
void (*PF0Task)(void);
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000001;//activate clock for Port F
	GPIO_PORTF_CR_R = 0x1F; // allow changes
	delay = SYSCTL_RCGC2_R; 		 //allow time for clock to start
	GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F 
  GPIO_PORTF_AMSEL_R &= ~0x01; //disable analog on PF0
	GPIO_PORTF_PCTL_R &= ~0x0000000f; //PCTL GPIO on PF0
	GPIO_PORTF_DIR_R &= ~0x01;  // PF0 is an input
	GPIO_PORTF_AFSEL_R &= ~0x01; //regular port function
	GPIO_PORTF_PUR_R |= 0x01; //pull-up on PF0
	GPIO_PORTF_DEN_R |= 0x01; //enable difital I/O on PF0
	GPIO_PORTF_IS_R &= ~0x01; //PF0 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x01; //PF0 is not both edges
	GPIO_PORTF_ICR_R = 0x01; //clear flag0
	GPIO_PORTF_IM_R |= 0x01; //enable interrupt on PF0
	LastPF0 = PF0;
	NVIC_PRI7_R = (NVIC_PRI7_R&0xff00ffff)|(priority<<21);//priority 5
	NVIC_EN0_R = NVIC_EN0_INT30;//ENABLE INTERRUPT 30
	PF0Task = task;
	return 1;
}

void static DebounceTaskPF0(void){
	OS_Sleep(10);
	LastPF0 = PF0;
	GPIO_PORTF_ICR_R = 0x01; //clear flag0
	GPIO_PORTF_IM_R |= 0x01; //enable interrupt on PF0
	OS_Kill();
}

//******** GPIOPortF_Handler*************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function

void GPIOPortF_Handler(void){
	if(((GPIO_PORTF_RIS_R>>4)&0x01)&&((GPIO_PORTF_MIS_R>>4)&0x01)){//PF4
		if(LastPF4 == 0){//rise edge
			(*PF4Task)();
		}
		GPIO_PORTF_IM_R &= ~0x10; //disarm interrupt on PF4
		OS_AddThread(&DebounceTaskPF4,128,1);
	}else if((GPIO_PORTF_RIS_R&0x01)&&(GPIO_PORTF_MIS_R&0x01)){//PF0
		if(LastPF0 == 0){//rise edge
			(*PF0Task)();
		}
		GPIO_PORTF_IM_R &= ~0x01; //disarm interrupt on PF0
		OS_AddThread(&DebounceTaskPF0,128,1);
	}
}
// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
	long status;
	status = StartCritical();
	RunPt->sleep = sleepTime;
	OS_Suspend();
	EndCritical(status);
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	tcbType *prevpt;
	tcbType *killpt;
	long status;

	status = StartCritical();
	//OS_DisableInterrupts();
	killpt = RunPt;
	prevpt = tail;
  //RunPt = RunPt->next;//RunPt now points to the next thread
	while(prevpt->next != killpt){
		prevpt = prevpt->next;
	}//find the thread which is before the current one
	if(killpt == tail){
		if(tail->next == tail){//only one thread
			tail = NULL;
		}else{
			prevpt->next = killpt->next;
			tail = prevpt;
		}
	}else{
		prevpt->next = killpt->next;//delete the current one in the linkedlist
	}
	//killpt->sp = NULL;
	//killpt->blocked = 0;
	//killpt->priority = 7;
	//killpt->sleep = 0;
	valid[killpt->id] = 0;
	killpt->id = -1;
	OS_Suspend();
	EndCritical(status);
	//OS_EnableInterrupts();
	
}

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size){
	//OS_InitSemaphore(&RoomLeft, FIFOSIZE);
	OS_InitSemaphore(&mutex,1);
	OS_InitSemaphore(&DataAvailable,0);
	PutPt = GetPt = &Fifo[0];
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
	long volatile *nextPutPt;//from book 
	nextPutPt = PutPt + 1;
	if(nextPutPt == &Fifo[FIFOSIZE]){
		nextPutPt = &Fifo[0];
	}
	if(nextPutPt == GetPt){
		return 0;//fail
	}else{
		*(PutPt) = data;
		PutPt = nextPutPt;
		OS_Signal(&DataAvailable);
		return 1;//success
	}
}  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
	unsigned long data;
	OS_Wait(&DataAvailable);
	OS_Wait(&mutex);
	data = *(GetPt++);
	if(GetPt == &Fifo[FIFOSIZE]){
		GetPt = &Fifo[0];
	}
	OS_Signal(&mutex);
	OS_Signal(&DataRoomLeft); 
	return data;
}


// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
	return DataAvailable.Value;
}

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
	OS_InitSemaphore(&DataValid,0);
	OS_InitSemaphore(&BoxFree,1);
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
	OS_bWait(&BoxFree);
	MailBox = data;
	OS_bSignal(&DataValid);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
	unsigned long data;
	OS_bWait(&DataValid);
	data = MailBox;
	OS_bSignal(&BoxFree);
	return data;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
	return TIMER1_TAV_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
	if(stop > start)
		return stop - start;
	else
		return start + (0xFFFFFFFF - stop);
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
	TIMER1_TAV_R = 0;
}


// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
	return TIMER1_TAV_R;
}
