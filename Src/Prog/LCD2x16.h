/*----------------------------------------------------------------------------
 * Name:    LCD.H
 * Purpose: LCD function prototypes
 * Version: V1.10
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * Copyright (c) 2005-2007 Keil Software. All rights reserved.
 *---------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void lcd_init       (void);
void lcd_clear      (void);
unsigned char lcd_read_status (void);
void lcd_putchar    (char c);
void set_cursor     (int column, int line);
void lcd_print      (char *string);
void lcd_bargraph   (int value, int size);
void lcd_bargraphXY (int pos_x, int pos_y, int value);

#ifdef __cplusplus
}
#endif

/* 
 * uncomment _LCD_USE_BUSY_STATUS_ if you want the LCD to use the busy flag
 * cannot works under simulation
 * if _LCD_USE_BUSY_STATUS_ is commented, the waitfucntion will use a delay fucntion 
 */

//#define _LCD_USE_BUSY_STATUS_
#define _LCD_SIMU_
/******************************************************************************/

