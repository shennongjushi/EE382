int ADC_Open(unsigned int channelNum);
void ADC0Seq3_Handler(void);
unsigned short ADC_In(void);
int ADC_Collect(unsigned int channelNum, unsigned int fs, void(*task)(unsigned long data));
void ADC_HW_Init(unsigned int channelNum, unsigned int prescale, unsigned int fs);