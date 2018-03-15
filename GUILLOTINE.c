#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include "stdio.h" 
#include "delay.h"
#include "nrf24.h"
#include "usart.h"
#include "mpu6050.h"
#include "i2cdriver.h"
#include <string.h>
#include "stdlib.h"
#define SAMPLES 100
uint32_t i,j,k;

uint8_t nRF24_payload[32];

nRF24_RXResult pipe;

uint8_t payload_length;
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF
char payloadch[300];
char message[32];
mpudata mpu1;
uint8_t buffer[14];
int meanAccX=0;
int meanAccY=0;
int meanAccZ=0;
int meanGyroX=0;
int meanGyroY=0;
int meanGyroZ=0;
int16_t GyroZMis;
int16_t AccXMis;
int16_t AccYMis; 
int16_t AccZMis;
uint32_t b,c,d,e,f,g;
int flameFlag,gasFlag,movFlag,panicFlag,doorFlag,secFlag=0;
int flameMutex,panicMutex=0;
void EXTI9_5_Config(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_Config(void);
void EXTI15_10_IRQHandler(void);
int alarmaS;
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00,
	nRF24_TX_SUCCESS,              
	nRF24_TX_TIMEOUT,               
	nRF24_TX_MAXRT                  
} nRF24_TXResult;

nRF24_TXResult tx_res;
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);
int main( )
{
	
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB->CRL = (GPIOB->CRL & 0xFFFFFF00) | 0x00000033; // PB0 as output
	//USARTTxString("THE MONEY STORE\r\n");
	//int c=0;
	init_i2c();
	mpu6050_init();
	
	USARTInit();
	USARTTxString("Starting Calibration of MPU6050..\r\n");
	char outputAG[100];//output for accelerometer and gyroscope measurements
	
	
	int i;
	//CALIBRATION OF GYROSCOPE AND ACCELEROMETER
	
	int tflag=0;//Trepidatory Flag
	int oflag=0;//Oscillatory Flag
	int sflag=0;//Sample flag
	int panicButtonFlag=0;
	int flameFlag=0;
	int doorFlag=0;
	int movFlag=0;
	
	
	for(i=0;i<SAMPLES;i++){
	I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, buffer, MPU6050_RA_ACCEL_XOUT_H, 14);
	}
	for(i=0;i<SAMPLES;i++){
	I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, buffer, MPU6050_RA_ACCEL_XOUT_H, 14);
			  meanAccX  +=(int)(buffer[0]  << 8)  | buffer[1];  
			  meanAccY  +=(int)(buffer[2]  << 8)  | buffer[3];
			  meanAccZ  +=(int)(buffer[4]  << 8)  | buffer[5];
			  mpu1.temp   =(int)(buffer[6]  << 8)  | buffer[7];
			  meanGyroX +=(int)(buffer[8]  << 8)  | buffer[9];
			  meanGyroY +=(int)(buffer[10]  << 8) | buffer[11];
			  meanGyroZ +=(int)(buffer[12]  << 8) | buffer[13];
		//sprintf(outputAG,"GX:%d GY:%d GZ:%d AX:%d AY:%d AZ:%d\r\n",meanGyroX,meanGyroY, meanGyroZ,meanAccX,meanAccY, meanAccZ);
			_delay_ms(5);
		USARTTxString(outputAG);
				
	}
	meanAccX/=SAMPLES;
	meanAccY/=SAMPLES;
	meanAccZ/=SAMPLES;
	meanGyroX/=SAMPLES;
	meanGyroY/=SAMPLES;
	meanGyroZ/=SAMPLES;
	
	
	_delay_ms(1000);
	I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, buffer, MPU6050_RA_ACCEL_XOUT_H, 14);
	GyroZMis =(((int16_t)(buffer[12]  << 8) | buffer[13])-(int16_t) meanGyroZ)/4;
	AccXMis =(((int16_t)(buffer[0]  << 8)  | buffer[1])-(int16_t) meanAccX)/8;
	AccYMis =(((int16_t)(buffer[2]  << 8) | buffer[13])-(int16_t) meanAccY)/8;
	AccZMis =mpu1.acc_z  =(((int16_t)(buffer[4]  << 8)  | buffer[5])-(int16_t) meanAccZ)/8;
	USARTTxString("Calibration finished, starting application\r\n");
	
	
	USARTInit();
	GPIO_InitTypeDef PORT;
    SPI_InitTypeDef SPI;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Mode  = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Pin   = nRF24_IRQ_PIN;
	GPIO_Init(nRF24_IRQ_PORT, &PORT);

	// Configure SPI pins (SPI2)
    PORT.GPIO_Mode  = GPIO_Mode_AF_PP;
    PORT.GPIO_Speed = GPIO_Speed_50MHz;
    PORT.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &PORT);


    // Initialize SPI2
    SPI.SPI_Mode = SPI_Mode_Master;
    SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI.SPI_CPOL = SPI_CPOL_Low;
    SPI.SPI_CPHA = SPI_CPHA_1Edge;
    SPI.SPI_CRCPolynomial = 7;
    SPI.SPI_DataSize = SPI_DataSize_8b;
    SPI.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(nRF24_SPI_PORT, &SPI);
    SPI_NSSInternalSoftwareConfig(nRF24_SPI_PORT, SPI_NSSInternalSoft_Set);
    SPI_Cmd(nRF24_SPI_PORT, ENABLE);



    // Initialize the nRF24L01 GPIO pins
    nRF24_GPIO_Init();

    // RX/TX disabled
    nRF24_CE_L();

    // Configure the nRF24L01+
    USARTTxString("nRF24L01+ check: ");
    if (!nRF24_Check()) {
    	USARTTxString("FAIL\r\n");
    	while (1);
    }
	USARTTxString("OK\r\n");

    // Initialize the nRF24L01 to its default state
    nRF24_Init();
	nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure TX PIPE
    static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address

    // Set TX power (maximum)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);

    // Set operational mode (PTX == transmitter)
    nRF24_SetOperationalMode(nRF24_MODE_TX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);


    // The main loop
    j = 0;
    payload_length = 31;
	
	
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//Enable TIM2 timer
	// delay = 50ms
	// PSC+1 = ceil(TIMCLK*delay/65535+1) = 300
	TIM2->PSC= 299;
	TIM2->ARR= 59999;
	TIM2->DIER |= TIM_DIER_UIE;//Enable TIM2 interruptions
	NVIC_EnableIRQ(TIM2_IRQn);		
	TIM2->CR1= TIM_CR1_CEN;//Enable TIM2 counter
	SCB->AIRCR = 0x5FA0000 | (5<< 8);
	EXTI9_5_Config(); 
	EXTI15_10_Config(); 
	USARTTxString("WALLOP!\r\n");
    while (1) {
    	
		}
}
    


nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	nRF24_WritePayload(pBuf, length);

	nRF24_CE_H();

	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	nRF24_CE_L();

	if (!wait) {
	
		return nRF24_TX_TIMEOUT;
	}


	USARTTxString("[");
	char statusch[100];
	sprintf(statusch,"%d",status);
	USARTTxString("]");

    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		return nRF24_TX_SUCCESS;
	}

	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

void TIM2_IRQHandler(void){//Handler for TIM2 interruption
	// Prepare data packet
    	/*for (i = 0; i < payload_length; i++) {
    		nRF24_payload[i] = j++;
    		if (j > 0x000000FF) j = 0;
    	}*/
		//Only placeholder GPIO
    	if( (GPIOA->IDR >>4 )& 0x1){
			alarmaS=1;
		 }
		else{
			GPIOB->ODR &=~(1<<1);
			 alarmaS=0;
		 }
		I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, buffer, MPU6050_RA_ACCEL_XOUT_H, 14);
			  mpu1.acc_x  =(((int16_t)(buffer[0]  << 8)  | buffer[1])-(int16_t) meanAccX)/8-AccXMis;  
			  mpu1.acc_y  =(((int16_t)(buffer[2]  << 8)  | buffer[3])-(int16_t) meanAccY)/8-AccYMis;
			  mpu1.acc_z  =(((int16_t)(buffer[4]  << 8)  | buffer[5])-(int16_t) meanAccZ)/8-AccZMis;
			  mpu1.temp   =(int16_t)(buffer[6]  << 8)  | buffer[7];
			  mpu1.gyro_x =(((int16_t)(buffer[8]  << 8)  | buffer[9])-(int16_t) meanGyroX)/4;
			  mpu1.gyro_y =(((int16_t)(buffer[10]  << 8) | buffer[11])-(int16_t) meanGyroY)/4;
			  mpu1.gyro_z =(((int16_t)(buffer[12]  << 8) | buffer[13])-(int16_t) meanGyroZ)/4-GyroZMis;
			  mpu1.temperature = (float)(mpu1.temp/340 +  36.53);
	
		//sprintf(message,"HOLA SOY EMMET\n");
		sprintf(message,"1 %05d %05d %05d %01d %01d %01d %01d %01d %01d\r",mpu1.acc_x,mpu1.acc_y, mpu1.acc_z,flameFlag,gasFlag,panicFlag,movFlag,doorFlag,secFlag);
		//sprintf(message,"holax");
		for (i = 0; i < payload_length; i++) {
    		nRF24_payload[i] =(uint8_t) message[i];
    	}
    	// Print a payload
    	USARTTxString("PAYLOAD:>");
		sprintf(payloadch,"%s",nRF24_payload);
    	USARTTxString(payloadch);
    	USARTTxString("< ... TX: ");

    	// Transmit a packet
    	tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
    	switch (tx_res) {
			case nRF24_TX_SUCCESS:
				USARTTxString("OK");
				break;
			case nRF24_TX_TIMEOUT:
				USARTTxString("TIMEOUT");
				break;
			case nRF24_TX_MAXRT:
				USARTTxString("MAX RETRANSMIT");
				break;
			default:
				USARTTxString("ERROR");
				break;
		}
    	USARTTxString("\r\n");

    	// Wait ~0.5s
		TIM2->SR &= ~TIM_SR_UIF;//Clear UIF Flag
    	//_delay_ms(500);

}
void EXTI9_5_Config(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Enable GPIOC clock	
  // Configure PB12 pin as input floating
  GPIOA->CRL = (GPIOA->CRL & 0x000FFFFF) | 0x44400000;
  // Enable AFIO clock (the selector of the input to EXTI12
  // is inside AFIO) 
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Connect EXTI12 Line to PB12 pin
  AFIO->EXTICR[1] |= 0<<4;
  AFIO->EXTICR[1] |= 0<<8;    
  AFIO->EXTICR[1] |= 0<<12;
  // Configure EXTI12 line: mode interrupt, trigger on rising
  EXTI->IMR  |= 7 << 5;  // Enable EXTIO interrupt   
  EXTI->RTSR |= 7 << 5; // detecting rising edge
 
 
  // Enable and set EXTI12 Interrupt to the lowest priority
    
  NVIC_EnableIRQ(EXTI9_5_IRQn);
   
  //NVIC_EnableIRQ(EXTI15_10_IRQn);
  
   SCB->AIRCR = 0x5FA0000 | (3<< 8);
  
}
void EXTI15_10_Config(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Enable GPIOC clock	
  // Configure PB12 pin as input floating
  GPIOA->CRH = (GPIOA->CRH & 0xFFF000FF) | 0x00044000;
  // Enable AFIO clock (the selector of the input to EXTI12
  // is inside AFIO) 
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Connect EXTI12 Line to PB12 pin
  AFIO->EXTICR[3] |= 0<<0;
  AFIO->EXTICR[3] |= 0<<12;		
  AFIO->EXTICR[2] |= 0<<12;    

  // Configure EXTI12 line: mode interrupt, trigger on rising
  EXTI->IMR  |= 3 << 11;  // Enable EXTIO interrupt   
  EXTI->IMR  |= 1 << 15;  // Enable EXTIO interrupt   	
  EXTI->RTSR |= 1 << 12; // detecting rising edge
  EXTI->RTSR |= 1 << 15; // detecting rising edge	
  EXTI->FTSR |= 1 << 11; // detecting rising edge	
 
 
  // Enable and set EXTI12 Interrupt to the lowest priority
    
  NVIC_EnableIRQ(EXTI15_10_IRQn);
   
  //NVIC_EnableIRQ(EXTI15_10_IRQn);
  
   SCB->AIRCR = 0x5FA0000 | (4<< 8);
  
}
void EXTI9_5_IRQHandler(void)//siempre activo
{
    

if (EXTI->PR & (1 << 5))
  {
  GPIOB->ODR |=(1<<0);
  _delay_ms(500);
    // Clear the EXTI line 0 pending bit	
	gasFlag=1;	
	panicMutex=1;
  EXTI->PR |= (1<< 5);  
  }
  if (EXTI->PR & (1 << 7))
  {
  GPIOB->ODR |=(1<<0);
  _delay_ms(500);
    // Clear the EXTI line 0 pending bit
	flameFlag=1;
	 panicMutex=1;
  EXTI->PR |= (1 << 7);  
  }
 if(EXTI->PR & (1 << 6)){
   GPIOB->ODR ^=(1<<0);
  _delay_ms(500);
    // Clear the EXTI line 0 pending bit
	 		 
	if(flameFlag || gasFlag){
		flameFlag=0;
		gasFlag=0;
	}else{
		if(panicFlag){
			panicFlag=0;
		}else{
			panicFlag=1;
			
		}
	}
  EXTI->PR |= (1<< 6);  
 }
else{
	
	 EXTI->PR |= (7<< 5);  
} 
}

void EXTI15_10_IRQHandler(void)//activacion por alarma
{
if(alarmaS==1){ 
	if(EXTI->PR & (1 << 12)){
	// Toggle PB0
  GPIOB->ODR |=(1<<1);
  _delay_ms(500);
    movFlag=1;
  EXTI->PR |= (1<< 12);  
	}
	if(EXTI->PR & (1 << 11)){
	// Toggle PB0
  GPIOB->ODR |=(1<<1);
  _delay_ms(500);
    doorFlag=1;
  EXTI->PR |= (1<< 11);  
	}
	
}
else{
	if(movFlag||doorFlag){
		movFlag=0;
		doorFlag=0;
	}else{
		movFlag=0;
		doorFlag=0;
	}	
EXTI->PR |= (3<< 11);
}

}
