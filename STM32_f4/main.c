#include "stm32f4xx.h"
#include "system_timetick.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#define		BUFF_SIZE			3
#define   T_s		0.012
#define   Start   'A'
#define   End     'E'
#define		BUFF_SIZE_Rx			11

uint8_t 	rxbuff[BUFF_SIZE_Rx] = {Start,0,0,0,0,0,0,0,0,'T',End};


int V =50;
int alpha  =150;
int beta = 150;

volatile float Kp,Ki,Kd,SP_speed;
static float e=0, e_sum=0,e_pre=0,d_e,Output=0,e_u=0,e_pre_pre=0;

uint8_t txbuff[3] ;
void init_main(void);
char mode ='T';



double Encoder ;
float output ;


GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;



void TIM2_Config(void);
void TIM8_ENCODER_Configuration(void);
void set_PWM(float duty, char mode);
void TIM_PWM_Config(void);
void DMA2_Stream0_IRQHandler(void);
float PID_Speed(float Kp, float Ki, float Kd, float setpoint, float speed);




//void delay_us(uint16_t period){
//  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
//  	TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
//  	TIM6->ARR = 10*period-1;
//  	TIM6->CNT = 0;
//  	TIM6->EGR = 1;		// update registers;

//  	TIM6->SR  = 0;		// clear overflow flag
//  	TIM6->CR1 = 1;		// enable Timer6

//  	while (!TIM6->SR);
//    
//  	TIM6->CR1 = 0;		// stop Timer6
//  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
//}

void delay_1ms(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
  	TIM6->ARR = 10*period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
		Encoder = (float)abs(TIM_GetCounter(TIM8)-32768)*1000*60/(12*4*374);
		//set_PWM(8);
		output = PID_Speed(2,15,0, V, Encoder);
		TIM_SetCounter(TIM8, 32768);
		TIM_Cmd(TIM8,ENABLE);
	
		set_PWM(output,mode);
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}


//void Change(char *x , int y ) {
//	 for(int i =2; i>-1 ; i--){
//	  x[i] = y%10;
//		y=y/10;	
//	 }
//	}

int main(void)
{
	/* Enable SysTick at 10ms interrupt */
	SysTick_Config(SystemCoreClock/100);
	TIM8_ENCODER_Configuration();
	TIM_PWM_Config();
	TIM2_Config();
	
	init_main();
	
	while(1){
		
			if (alpha > beta){
			 beta += 2;			
			}
			else if(alpha <beta){
			 beta -= 2;
			}
			else {
			
				beta = alpha;
			}

			TIM2->CCR2= beta;
			delay_1ms(12);

//			tick_count = 0; // phai du 3 buoc duoi cho DMA. 1 la xoa co, 2 la gan byte truyen, 3 la cho phep DMA bat dau truyen
//			DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4); //moi lan DMA hoan thanh thi DMA_FLAG_TCIFx bat len 1, phai xoa co nay cho lan ke tiep
//			DMA1_Stream4->NDTR = BUFF_SIZE; //muon truyen bao nhieu byte nap vo NDTR, vd 50 bytes, NDTR = 50;moi lan truyen xong 1 byte thi NDTR--
//			DMA_Cmd(DMA1_Stream4, ENABLE); //bat dau cho DMA truyen
//			
	}
	
}




void set_PWM(float duty,char mode)
{

	if(mode == 'N') // 
	{
	TIM4->CCR1=duty*4199/100;
	TIM4->CCR2=0*4199/100;

	}
  else if(mode == 'T')// 
	{
    TIM4->CCR1=0*4199/100;
  	TIM4->CCR2=duty*4199/100;
	}
	else if(mode == 'S')// Stop
	{
	TIM4->CCR1=0*4199/100;
  TIM4->CCR2=0*4199/100;
	e=0, e_sum=0,e_pre=0,Output=0,e_u=0,e_pre_pre=0;
	}
}

float PID_Speed(float Kp, float Ki, float Kd, float setpoint, float speed)
{ e_pre_pre = e_pre;
	e_pre=e;
	e=setpoint-speed;
	e_u = Output;
	Output=e_u+Kp*(e-e_pre)+Ki*((float)T_s/2)*(e+e_pre)+(Kd/(float)T_s)*(e-2*e_pre+e_pre_pre);
	
	if(Output >100)
		Output=100;
	else if(Output<0)
	{
		Output=0;
	}
	return(Output);
}

void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
	DMA_InitTypeDef  	DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure; 
	ADC_CommonInitTypeDef  ADC_CommonInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;	
  /* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
/* Enable ADC1 clock */		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4); 

  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(UART4, &USART_InitStructure);
	
	/* Enable USART */
  USART_Cmd(UART4, ENABLE);
	
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
	
	
	/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE_Rx;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream2, ENABLE);

  
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	
}


void TIM_PWM_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	
	TIM_TimeBaseStructure.TIM_Prescaler=0; //+1
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=4200-1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=0;

	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2PolarityConfig(TIM4,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
}
void TIM2_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	
	TIM_TimeBaseStructure.TIM_Prescaler=839; 
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=1999;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse=0;

	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2PolarityConfig(TIM2,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}
void TIM8_ENCODER_Configuration(void)
{
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStruct;
GPIO_InitTypeDef  GPIO_InitStructure;
//Configure peripheral clocks
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
//Configure pins
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
GPIO_Init(GPIOC, &GPIO_InitStructure);
GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);  //C6 -TIM8_CH1
GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);  //C7  TIM8_CH2
//Configure Timer
TIM_TimeBaseStructure.TIM_Prescaler =0;
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_Period = 65535;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
//Debounce filter
TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
TIM_ICInitStruct.TIM_ICFilter=4;
TIM_ICInit(TIM8, &TIM_ICInitStruct);
TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
TIM_ICInitStruct.TIM_ICFilter=4;
TIM_ICInit(TIM8, &TIM_ICInitStruct);

//Setup quadrature encoder and enable timer
TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);
TIM_SetCounter(TIM8, 32768);
TIM_Cmd(TIM8, ENABLE); 
}



void DMA1_Stream2_IRQHandler(void)
{
  uint16_t i;

  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
  if (rxbuff[0] == Start && rxbuff[10] == End) 
				{for(i=1; i<BUFF_SIZE_Rx-2; i++){
					 rxbuff[i]=rxbuff[i]-0x30;} 
				V= rxbuff[1]*1000+rxbuff[2]*100+rxbuff[3]*10+rxbuff[4]*1;
				alpha= rxbuff[5]*1000+rxbuff[6]*100+rxbuff[7]*10+rxbuff[8]*1;
				//if (alpha >= 190) alpha = 190;
				//if (alpha <= 70) alpha = 70;
				//TIM2->CCR2= alpha;
				mode =rxbuff[9];	
				//delay_us(50);
				DMA_Cmd(DMA1_Stream2, ENABLE);}
	else 
	{
				V = 0;  	
	}
}