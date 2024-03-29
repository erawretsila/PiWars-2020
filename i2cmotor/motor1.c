/*
 * keyboard
 *
 * Copyright 2016 Alister <alister.ware@ntlworld.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Register Map
 * 00   Motor 1 Speed
 * 01   Moteo 2 Speed
 * 02   Motor 1 Dir  - 00 stop, 01 = forward 02 - reverse, 03 brake
 * 03   Motor 2 Dir
 * 04   Servo 1     //16 bit Value
 * 05   Servo 1     //16 bit value
 * 06   Servo 2    // Future Expansion Not Yet Implemented
 * 07   Servo 2
 *
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "usi_i2c_slave.h"
#include "motor.h"

uint8_t serial_i2c_buffer[8];
uint8_t serial_i2c_buffer_pos;

extern uint8_t usi_i2c_slave_address;
extern uint8_t *USI_Slave_register_buffer[];

ISR(TIMER0_OVF_vect){
    PINA=0x01;
}

int main(void) {
    uint8_t motor1 =0x55;
    uint8_t motor2 =0xaa;
    PORTA=0xff; //turn on pulups
    uint8_t I2CAddr =0x63; //(PINA & 0x03) |0x60;  //read pins PA1 & PA2 for ID 
// Initialise Motor 1 & Motor 2
    Motor1_DDR |= _BV(M1PWM)|_BV(M1X)|_BV(M1Y);
    Motor2_DDR |= _BV(M2PWM)|_BV(M2X)|_BV(M2Y);
    TCCR0A = 2<<COM0A0 |2 <<COM0B0 | 3<<WGM00;
    TCCR0B = 5<<CS00;
    DDRB |=_BV(PB3) | _BV(PB4);
//initialise Servo 1 & Servo 2
	TCCR1A = 0| _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
	TCCR1B = 0 | _BV(WGM13) |_BV(WGM12) | _BV(CS11);
	ICR1 =20000;
	OCR1A=1500;
	OCR1B=1500;
	
	USI_Slave_register_buffer[0]=&OCR0A;
	USI_Slave_register_buffer[1]=&OCR0B;
    USI_Slave_register_buffer[2]=&motor1;
    USI_Slave_register_buffer[3]=&motor2;
	USI_Slave_register_buffer[4]=&OCR1AH;
	USI_Slave_register_buffer[5]=&OCR1AL;
	USI_Slave_register_buffer[6]=&OCR1BH;
	USI_Slave_register_buffer[7]=&OCR1BL;
//initialise I2C
    USI_I2C_Init(I2CAddr);
    
    sei();  //enable interupts!
    set_sleep_mode(SLEEP_MODE_IDLE);
    while (1){
//        sleep_enable();
      //        sleep_cpu();
//        OCR1AL = USI_Slave_register_buffer[5] &0xff;
        PORTB = (PINB & ~0x03) | (motor1 &0x03);
        PORTD = (PIND & ~0x03)
         | (motor2 &0x03);
        PINA=0x01;
    }
}
