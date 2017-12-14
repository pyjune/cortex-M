/*
 * lcd.c
 *
 *  Created on: 2017. 12. 14.
 *      Author: user
 */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lcd.h"

static void LCD_cmdH(int cmd);
static void LCD_cmdL(int cmd);

static void LCD_cmdH(int cmd)
{
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_RS, TLCD_CMD);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D7, (cmd&(1<<7))>>7);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D6, (cmd&(1<<6))>>6);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D5, (cmd&(1<<5))>>5);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D4, (cmd&(1<<4))>>4);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_E, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_E, GPIO_PIN_RESET);


}
static void LCD_cmdL(int cmd)
{
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_RS, TLCD_CMD);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D7, (cmd&(1<<3))>>3);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D6, (cmd&(1<<2))>>2);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D5, (cmd&(1<<1))>>1);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_D4, (cmd&(1<<0))>>0);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_E, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TLCD_GPIO, TLCD_E, GPIO_PIN_RESET);
}

void LCD_init(void)
{
	//Function Set 0x20
	LCD_cmdH(0x30);
	HAL_Delay(5); // wait 4.1ms
	LCD_cmdH(0x30);
	// wait 100us
	LCD_cmdH(0x32);
	LCD_cmdL(0x32);
	LCD_cmdH(0x28);
	LCD_cmdL(0x28);
	LCD_cmdH(0x0f); //display on, cursor on , blinking on
	LCD_cmdL(0x0f);
	LCD_cmdH(0x01);
	LCD_cmdL(0x01);
	LCD_cmdH(0x06);
	LCD_cmdL(0x06);
	LCD_cmdH(0x02);
	LCD_cmdL(0x02);
}
void LCD_setCursor(int col, int row)
{

}
void LCD_print(char *data)
{

}
