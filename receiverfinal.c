#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include "stdio.h" 
#include "delay.h"
#include "nrf24.h"
#include "usart.h"
#include <string.h>
#include "stdlib.h"
uint32_t i,j,k;

uint8_t nRF24_payload[32];

nRF24_RXResult pipe;

uint8_t payload_length;
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF
char payloadch[300];

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
    // This is simple receiver with one RX pipe:
    //   - pipe#1 address: '0xE7 0x1C 0xE3'
    //   - payload: 5 bytes
    //   - RF channel: 115 (2515MHz)
    //   - data rate: 250kbps (minimum possible, to increase reception reliability)
    //   - CRC scheme: 2 byte

    // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(120);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_250kbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { 0xE8, 0x1D , 0xE4 };
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 31); // Auto-ACK: disabled, payload length: 5 bytes

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//Enable TIM2 timer
	// delay = 50ms
	// PSC+1 = ceil(TIMCLK*delay/65535+1) = 300
	TIM2->PSC= 299;
	TIM2->ARR= 59999;
	TIM2->DIER |= TIM_DIER_UIE;//Enable TIM2 interruptions
	NVIC_EnableIRQ(TIM2_IRQn);		
	TIM2->CR1= TIM_CR1_CEN;//Enable TIM2 counter

    // The main loop
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
void TIM2_IRQHandler(void){
	//
        // Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
        //
        // This is far from best solution, but it's ok for testing purposes
        // More smart way is to use the IRQ pin :)
        //
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
            // Get a payload from the transceiver
            pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();

            // Print a payload contents to UART
            //USARTTxString("RCV PIPE#");
            //USARTTxString((char *)pipe);
            //USARTTxString(" PAYLOAD:>");
            USARTTxString((char *)nRF24_payload);
            USARTTxString("\r\n");
        }
		TIM2->SR &= ~TIM_SR_UIF;//Clear UIF Flag
}