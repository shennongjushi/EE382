#ifndef __OS_H
#define __OS_H  1

int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority);

void OS_ClearPeriodicTime(void);

unsigned long OS_ReadPeriodicTime(void);

#endif
