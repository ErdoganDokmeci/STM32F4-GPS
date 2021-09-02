// GY-NEO6M GPS Application using STM32F446RE-Nucleo
// This program was tested with Keil uVision v5.33.0.0 with DFP v2.15.0
// By default, the clock is running at 16 MHz.
// 02 Sep 2021, Eskisehir

#include <stdio.h>
#include "stm32f4xx.h"
#include "gps.h"
#include "fonts.h"
#include "ssd1306.h"

void USART2_init (void);
void USART2_write (int ch);

void UART4_init (void);
void UART4_write (int ch);
char UART4_read(void);

void I2C1_init(void);										
void I2C1_burstWrite(char saddr, int n, uint8_t* data);		// Burst write
void delayMs(int n);

uint8_t rx_data;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
uint8_t parse_err;

int main(void)
{
	__disable_irq();
	USART2_init();          							// initialize USART2
	UART4_init(); 												// Initialize UART4 for GPS data	

	I2C1_init();													// Initialize I2C1					
	SSD1306_Init();	
	SSD1306_Fill(0);											// Clear all the pixels	
	SSD1306_GotoXY(0,0);	
	
	UART4->CR1 |= 0x0020;
	NVIC_EnableIRQ(UART4_IRQn);
	__enable_irq();
		
	char buff[26];
		
	while(1) {          									// Loop forever
		SSD1306_Fill(0);										// Clear all the pixels	
		SSD1306_GotoXY(0,0);		
		int i;

		sprintf(buff, "%.6f %.6f\n", latitude, longitude);		

		for (i = 0; i < strlen(buff); i++) {
			if (buff[i] == ' ') {
				SSD1306_GotoXY (0, 30);
				continue;
			}
			if (buff[i] == '\n')
				break;
			SSD1306_Putc(buff[i], &Font_11x18, 1);			
			//USART2_write(buff[i]);			
		}
		
		SSD1306_UpdateScreen(); 							// Update display	
		delayMs(1000);
	}
}

/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2 for USART2_TX */
    GPIOA->AFR[0] &= ~0x0F00;
    GPIOA->AFR[0] |=  0x0700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x0030;
    GPIOA->MODER  |=  0x0020;   /* enable alternate function for PA2 */

    USART2->BRR = 0x0683;       /* 9600 baud @ 16 MHz */
    USART2->CR1 = 0x0008;       /* enable Tx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}

/* Write a character to USART2 */
void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (ch & 0xFF);
}

/* initialize UART4 to receive/transmit at 9600 Baud */
void UART4_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x80000;    /* Enable UART4 clock */

    /* Configure PA0, PA1 for UART4 TX, RX */
    GPIOA->AFR[0] &= ~0x00FF;
    GPIOA->AFR[0] |=  0x0088;   /* alt8 for UART4 */
    GPIOA->MODER  &= ~0x000F;
    GPIOA->MODER  |=  0x000A;   /* enable alternate function for PA0, PA1 */

    UART4->BRR = 0x0683;        /* 9600 baud @ 16 MHz */
    UART4->CR1 = 0x000C;        /* enable Tx, Rx, 8-bit data */
    UART4->CR2 = 0x0000;        /* 1 stop bit */
    UART4->CR3 = 0x0000;        /* no flow control */
    UART4->CR1 |= 0x2000;       /* enable UART4 */
}

/* Write a character to UART4 */
void UART4_write (int ch) {
    while (!(UART4->SR & 0x0080)) {}   // wait until Tx buffer empty
    UART4->DR = (ch & 0xFF);
}

/* Read a character from UART4 */
char UART4_read(void) {
    while (!(UART4->SR & 0x0020)) {}   // wait until char arrives
    return UART4->DR;
}

void UART4_IRQHandler(void) {
    if (UART4->SR & 0x0020) {
        rx_data = UART4->DR;             /* Read a character from UART4 */
        if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
					rx_buffer[rx_index++] = rx_data;
				} 
				else {
					if(GPS_validate((char*) rx_buffer))
						GPS_parse((char*) rx_buffer);
					rx_index = 0;
					memset(rx_buffer, 0, sizeof(rx_buffer));
				}
    }
}

void I2C1_init(void) {
	RCC->AHB1ENR |= 2;										// Enable GPIOB clock
	RCC->APB1ENR |= 0x00200000;						// Enable I2C1 clock
	
	// Configure PB8, PB9 pins for I2C1
	GPIOB->MODER  &= ~0x000F0000;					// Reset PB8, PB9
	GPIOB->MODER  |= 0x000A0000;					// PB8, PB9 use alternate function
	GPIOB->AFR[1] &= ~0x000000FF;
	GPIOB->AFR[1] |= 0x00000044;					// PB8, PB9 I2C1 SCL, SDA
	GPIOB->OTYPER |= 0x00000300;					// Output open-drain
	GPIOB->PUPDR  &= ~0x000F0000;
	GPIOB->PUPDR  |= 0x00050000;					// With pull-up
	
	I2C1->CR1   = 0x8000;									// Software reset I2C1
	I2C1->CR1  &= ~0x8000;								// Out of reset
	I2C1->CR2   = 0x0010;									// Peripheral clock is 16 MHz
	I2C1->CCR   = 0x800E;									// Fast mode
	I2C1->TRISE = 0x5;										// Maximum rise time
	I2C1->CR1  |= 0x0001;									// Enable I2C1 module	
}

void I2C1_burstWrite(char saddr, int n, uint8_t* data) {
	int i;
	volatile int tmp;
	
	while(I2C1->SR2 & 2);									// Wait until bus not busy
	I2C1->CR1 &= ~0x800;									// Disable POS
	I2C1->CR1 |= 0x100;										// Generate start
	while(!(I2C1->SR1 & 1));							// Wait until start flag is set
	I2C1->DR = saddr << 1;								// Transmit slave addr
	while(!(I2C1->SR1 & 2));							// Wait until addr flag is set
	tmp = I2C1->SR2;											// Clear addr flag
	while(!(I2C1->SR1 & 0x80));						// Wait until data register empty
		
	// Write all the data
	for(i = 0; i < n; i++) {
		while(!(I2C1->SR1 & 0x80));					// Wait until data register empty
		I2C1->DR = *data++;									// Transmit data
	}
	
	while(!(I2C1->SR1 & 4));							// Wait until transfer finished
	I2C1->CR1 |= 0x200;										// Generate stop
}

void delayMs(int n) {
	int i;
	SysTick->LOAD = 16000-1;
	SysTick->VAL  = 0;
	SysTick->CTRL = 0x5;
	for(i = 0; i < n; i++) {
		while((SysTick->CTRL & 0x10000) == 0);	// Wait until the COUNTFLAG is set
	}
	SysTick->CTRL = 0;										// Stop the timer
}
