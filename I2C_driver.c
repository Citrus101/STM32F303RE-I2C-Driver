#include "I2C_driver.h"

void delay(uint32_t time){
	for(int i=0;i<time;i++) (void) 0;
}

void gpio_set_mode(PIN_t pin, uint8_t mode){

    GPIO_TypeDef *gpio = GPIO(PINBANK(pin)); 			// select gpio bank

    gpio->MODER &= ~(3U << (PINNO(pin) * 2));         	// clear pin mode
    gpio->MODER |= ((mode & 3U) << (PINNO(pin) * 2)); 	// set pin mode
}

void gpio_set_af(PIN_t pin, uint8_t mode){

    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNO(pin);
    gpio -> AFR[n >> 3] &= (~(0xf << (n * 4)));           //clear pin mode
    gpio -> AFR[n >> 3] |= ((mode & 0xfU) << (n * 4));    //set pin mode
}

void gpio_write(uint16_t pin, bool val)
{
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNO(pin);
    gpio->BSRR = (1U << n << (val ? 0 : 16));
}

void I2C_init(I2C_TypeDef *I2C, uint16_t own_address){

	PIN_t SDA, SCL;

	if(I2C == I2C1){

		 RCC -> APB1ENR |= BIT(21); 	//Enable I2C clock
		 SDA = PIN('A', 14);
		 SCL = PIN('A', 15);

	}else if(I2C == I2C2){

		 RCC -> APB1ENR |= BIT(22);		//Enable I2C clock
		 SDA = PIN('A', 9);
		 SCL = PIN('A', 10);

	}

	RCC -> AHBENR 	|= BIT(22);		//Enable GPIOA clock
	gpio_set_mode(SCL, MODE_AF);
	gpio_set_mode(SDA, MODE_AF);

	//Setting to I2C_SDA and I2C_SCL
	gpio_set_af(SDA, 4);
	gpio_set_af(SCL, 4);

	//Pull Up resistors
	GPIOA -> PUPDR |= (2 << (PINNO(SDA) * 2));
	GPIOA -> PUPDR |= (2 << (PINNO(SCL) * 2));

	//Open Drain Mode
	GPIOA -> OTYPER |= (1 << PINNO(SDA));
	GPIOA -> OTYPER |= (1 << PINNO(SCL));

	I2C -> CR1 &= ~(1 << 0); //Disabling PE
	(void) 0; //Stalling for 3 cycles

	I2C -> CR1 &= ~(1 << 17); //Enabling Clock stretching
	I2C -> CR1 &= ~(1 << 12); //Enabling analog noise filter

	//Setting up 100Khz standard mode
	I2C -> TIMINGR = (0x1 << 28); 	//PRESCALE
	I2C -> TIMINGR = (0x13 << 0); 	//SCLL
	I2C -> TIMINGR = (0xf << 8);	//SCLH
	I2C -> TIMINGR = (0x2 << 16);	//SDADEL
	I2C -> TIMINGR = (0x4 << 20);	//SCLDEL

	//Configuring Own Address
	I2C -> OAR1 &= ~(1 << 15);			//Disabling Own Address
	I2C -> OAR1 &= ~(1 << 10);			//7 bit Address
	I2C -> OAR1 |= 	(own_address << 1); //loading Own Address
	I2C -> OAR1 |=  (1 << 15);			//Enabling Own Address

	I2C -> ISR 	|= 	(1<<0);				//flush the transmit register

}

int I2C_scanner (I2C_TypeDef *I2C, uint8_t slave_addr){

	PIN_t SDA, SCL;
	int ack;

	if(I2C == I2C1){

		SDA = PIN('A', 14);
		SCL = PIN('A', 15);

	}else if(I2C == I2C2){

		SDA = PIN('A', 9);
		SCL = PIN('A', 10);

	}

	I2C -> ICR |=  	(I2C_ICR_ADDRCF); 	//Clearing Start bit, ADDRCF = 1
	I2C -> CR2 |=	(slave_addr << 1);	//Loading slave address
	I2C -> CR2 |=	(I2C_CR2_RD_WRN);	//Transfer direction set to transmit/write
	I2C -> CR2 |= 	(I2C_CR2_AUTOEND);	//Stop condition sent when N bytes have been transferred
	I2C -> CR2 &= 	~(I2C_CR2_RELOAD);	//Disable Reload
	I2C -> CR2 |=	(I2C_CR2_NBYTES & (0 << I2C_CR2_NBYTES_Pos)); //0 Byte being transferred for test

	while(!((GPIOA -> IDR | PINNO(SDA)) || (GPIOA -> IDR | PINNO(SCL)))) (void) 0; //To make sure the SDA and SCL pins are idle

	I2C -> CR1 	|= 	(1 << 0);			//Enabling PE
	(void) 0; 							//Stalling for 3 cycles

	I2C -> CR2 |=	(I2C_CR2_START);


	if((I2C -> ISR & I2C_ISR_NACKF) == (I2C_ISR_NACKF)){

		I2C -> CR1 &= ~(I2C_CR1_PE);
		ack = 0;

	}else{

		I2C -> CR1 &= ~(I2C_CR1_PE);
		ack = 1;

	}

	return ack;
}

int I2C_tx(I2C_TypeDef *I2C, uint8_t slave_addr, uint8_t number_of_bytes, uint8_t *bytes_to_send){

	PIN_t SDA, SCL;
	int ack;

	if(I2C == I2C1){

		SDA = PIN('A', 14);
		SCL = PIN('A', 15);

	}else if(I2C == I2C2){

		SDA = PIN('A', 9);
		SCL = PIN('A', 10);

	}

	I2C -> ICR |=  	(I2C_ICR_ADDRCF); 	//Clearing Start bit, ADDRCF = 1
	I2C -> CR2 |=	(slave_addr << 1);	//Loading slave address
	I2C -> CR2 |=	(I2C_CR2_RD_WRN);	//Transfer direction set to transmit/write
	I2C -> CR2 |= 	(I2C_CR2_AUTOEND);	//Stop condition sent when N bytes have been transferred
	I2C -> CR2 &=  	~(I2C_CR2_RELOAD);	//Disable Reload
	I2C -> CR2 |=	(I2C_CR2_NBYTES & (number_of_bytes << I2C_CR2_NBYTES_Pos)); //0 Byte being transferred for test

	while(!((GPIOA -> IDR | PINNO(SDA)) || (GPIOA -> IDR | PINNO(SCL)))) (void) 0; //To make sure the SDA and SCL pins are idle

	I2C -> CR1 	|= 	(1 << 0);			//Enabling PE
		(void) 0; 							//Stalling for 3 cycles

	I2C -> CR2 |=	(I2C_CR2_START);



	if((I2C -> ISR & I2C_ISR_NACKF) == (I2C_ISR_NACKF)){ //If message is not aknowledged
		I2C -> CR1 &= ~(I2C_CR1_PE);
		ack = 0;

	}else{
		while(number_of_bytes){
			while(I2C -> ISR & I2C_ISR_TXE){ //Check if the transmit buffer is empty
				I2C -> TXDR = (volatile uint8_t ) *bytes_to_send++;
				number_of_bytes--;
				delay(0xff);
			}I2C -> ISR |= (I2C_ISR_TXE);	//Forcefully flush the transmit buffer
		}
		I2C -> CR1 &= ~(I2C_CR1_PE);
		ack = 1;
	}

	return ack;
}

void I2C_rx(I2C_TypeDef *I2C, uint8_t slave_addr, uint8_t n, uint8_t *buffer){
	PIN_t SDA, SCL;


	if(I2C == I2C1){

		SDA = PIN('A', 14);
		SCL = PIN('A', 15);

	}else if(I2C == I2C2){

		SDA = PIN('A', 9);
		SCL = PIN('A', 10);

	}

	I2C -> ICR |=  	(I2C_ICR_ADDRCF); 	//Clearing Start bit, ADDRCF = 1
	I2C -> CR2 |=	(slave_addr << 1);	//Loading slave address
	I2C -> CR2 &=	~(I2C_CR2_RD_WRN);	//Transfer direction set to recieve/read
	I2C -> CR2 |= 	(I2C_CR2_AUTOEND);	//Stop condition sent when N bytes have been transferred
	I2C -> CR2 &= 	~(I2C_CR2_RELOAD);	//Disable Reload
	I2C -> CR2 |=	(I2C_CR2_NBYTES & (0 << I2C_CR2_NBYTES_Pos)); //0 Byte being transferred for test

	while(!((GPIOA -> IDR | PINNO(SDA)) || (GPIOA -> IDR | PINNO(SCL)))) (void) 0; //To make sure the SDA and SCL pins are idle

	I2C -> CR1 	|= 	(I2C_CR1_PE);			//Enabling PE
	(void) 0; 							//Stalling for 3 cycles

	I2C -> CR2 |=	(I2C_CR2_START);

	while(n){
		if(I2C -> ISR & I2C_ISR_RXNE){
			*(buffer++) = I2C -> RXDR;
			n--;
		}
	}

	I2C -> CR1 &= ~(I2C_CR1_PE);

}

