#include "inc/tm4c123gh6pm.h"
#include "OS.h"
#include "UART.h"
#include "FIFO.h"
#include "PLL.h"


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void); // user function

int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority) {
	long sr;
	unsigned long PriorityHex;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x02;;   // 0) activate TIMER1
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

void OS_ClearPeriodicTime(void) {
	TIMER1_TAV_R = 0;
}

unsigned long OS_ReadPeriodicTime(void) {
	unsigned long CurrentTime;
	CurrentTime = TIMER1_TAV_R;
	return CurrentTime;
}
