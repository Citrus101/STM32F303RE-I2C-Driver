#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f3xx.h"

#define GPIO(bank) ((GPIO_TypeDef *)(0x48000000 + 0x400 * (bank)))

#define BIT(x) (1UL << x)
#define PIN(bank, x) (((bank - 'A') << 8) | (x))
#define PINNO(x) (x & 255)
#define PINBANK(x) (x >> 8)

typedef uint16_t PIN_t;

void gpio_set_mode(uint16_t pin, uint8_t mode);
void gpio_write(uint16_t pin, bool val);
void gpio_set_af(uint16_t pin, uint8_t mode);

void I2C_init(I2C_TypeDef *I2C, uint16_t own_address);
int  I2C_scanner(I2C_TypeDef *I2C, uint8_t slave_addr);
int  I2C_tx(I2C_TypeDef *I2C, uint8_t slave_addr, uint8_t number_of_bytes, uint8_t *bytes_to_send);
void I2C_rx(I2C_TypeDef *I2C, uint8_t slave_addr, uint8_t n, uint8_t *buffer);

void delay();

#endif /* INC_I2C_DRIVER_H_ */
