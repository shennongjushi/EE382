int ADC_Open(unsigned int channelNum);
void ADC0Seq3_Handler(void);
unsigned short ADC_In(void);
int ADC_Collect(unsigned int channelNum, unsigned int fs, unsigned short buffer[], unsigned int numberOfSamples);
int ADC_Status(void);
void ADC_Init(unsigned int channelNum, unsigned int prescale, unsigned int fs);
