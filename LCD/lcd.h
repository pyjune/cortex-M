/*
 * lcd.h
 *
 *  Created on: 2017. 12. 14.
 *      Author: user
 */

#ifndef LCD_H_
#define LCD_H_

#define TLCD_GPIO	GPIOC
#define TLCD_RS	GPIO_PIN_8
#define TLCD_E	GPIO_PIN_9
#define TLCD_D4	GPIO_PIN_12
#define TLCD_D5	GPIO_PIN_13
#define TLCD_D6 GPIO_PIN_14
#define TLCD_D7	GPIO_PIN_15

#define TLCD_CMD	GPIO_PIN_RESET
#define TLCD_DATA	GPIO_PIN_SET

void LCD_init(void);
void LCD_setCursor(int col, int row);
void LCD_print(char *data);
void LCD_putCh(int data);
void LCD_clear(void);

#endif /* LCD_H_ */