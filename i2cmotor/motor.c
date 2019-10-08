/*
 * Z-Gun
 *
 * Copyright 2019 Alister <alister.ware@ntlworld.com>
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
 * 04   Servo 1 high
 * 05   Servo 1 low
 * 06   Servo 2 high
 * 07   Servo 2 low
 * Port D - Care req bits D3,D4 & D5 in use for Motor 2
 * 08   DDRD    Data Direction register
 * 09   PORTD   Output Port
 * 0a   PIND    Input port
 * 0f   OSCAL - Oscillator calibration
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include "usi_i2c_slave.h"
#include "motor.h"


EEMEM   char version[] = "I2C Motor & Servo Controller V2.2";
EEMEM   char copyright[]="(c)A Ware 2019";
EEMEM   uint8_t oscal =0xff;
EEMEM   uint8_t i2caddr =0xff;


uint8_t serial_i2c_buffer[8];
uint8_t serial_i2c_buffer_pos;

extern uint8_t usi_i2c_slave_address;
extern uint8_t* USI_Slave_register_buffer[];

ISR(TIMER0_OVF_vect){
}

void singleShot(uint8_t *motor,uint8_t sense){
        if (SENSE & _BV(sense)) { // Mode 1 - slide closed wait for open
//            PINA =_BV(PA1);
            if (PINA & _BV(sense)) { //slide open
                SENSE &= ~(_BV(sense)); // Set Mode 0
                _delay_ms(60);   //debounce
            }
        } else {        //slide open wait for close & disable motor
            if (!(PINA & _BV(sense))){  //Slide closed
                SENSE |=_BV(sense); //Set Mode 1
                *motor=0;   //Motor Off
                _delay_ms(60);   //debounce
            }
        }

}

int main(void) {
    uint8_t Motor1;
    uint8_t Motor2;
    uint8_t i2c_addr;
    uint8_t fcal;
    wdt_reset();
    wdt_enable(WDTO_1S);
    PORTA=0xff; //turn on pulups
    DDRA=0x03;  //set port a input (should be unnecessary
    
// Initialise Motor 1 & Motor 2
    Motor1_DDR |= _BV(M1PWM)|_BV(M1X)|_BV(M1Y);
    Motor2_DDR |= _BV(M2PWM)|_BV(M2X)|_BV(M2Y);
    //initialise timer 0 as 8bit PWM for motor speed
    TCCR0A = 2<<COM0A0 |2 <<COM0B0 | 3<<WGM00;
    TCCR0B = 5<<CS00;
    DDRB |=_BV(SERVO1) | _BV(SERVO2);
//initialise Servo 1 & Servo 2 16 bit timer 0 Approx 1 micro second resolution
	TCCR1A = 0| _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
	TCCR1B = 0 | _BV(WGM13) |_BV(WGM12) | _BV(CS11);
	ICR1 =20000;    //20000 uS = 20mS = 50hz
	OCR1A=1000;
	OCR1B=1000;

    USI_Slave_register_buffer[0]=&OCR0A;
    USI_Slave_register_buffer[1]=&OCR0B;
    USI_Slave_register_buffer[2]=&Motor1;
    USI_Slave_register_buffer[3]=&Motor2;
    USI_Slave_register_buffer[4]=&OCR1AH;
    USI_Slave_register_buffer[5]=&OCR1AL;
    USI_Slave_register_buffer[6]=&OCR1BH;
    USI_Slave_register_buffer[7]=&OCR1BL;
    USI_Slave_register_buffer[8]=&DDRD;
    USI_Slave_register_buffer[9]=&PORTD;
    USI_Slave_register_buffer[0x0a]=&PIND;
    USI_Slave_register_buffer[0x0f]=&OSCCAL;
    
//initialise OSCCAL
    fcal=eeprom_read_byte(&oscal);
    if (fcal==0xff){    //blank value
        fcal=OSCCAL;
        eeprom_update_byte(&oscal,fcal);
    } else {
        OSCCAL=fcal;
    }   
//initialise I2C
    i2c_addr=eeprom_read_byte(&i2caddr);    //read I2C addr from EEPROM
    if (i2c_addr==0xff){    //I2C Addr not set
        i2c_addr=0x60;  //default addr
        eeprom_update_byte(&i2caddr,i2c_addr); //write to eeprom
    }
    USI_I2C_Init(i2c_addr);
    
    sei();  //enable interupts!
    while (1){
//        PINA=0x01;
        if (PIN_USI & (1 << PIN_USI_SCL)){
            wdt_reset();
        }
        if (Motor1 & 1) Motor1_Port |= _BV(M1X); else Motor1_Port &=~(_BV(M1X));
        if (Motor1 & 2) Motor2_Port |= _BV(M1Y); else Motor1_Port &=~(_BV(M1Y));
        if (Motor2 & 1) Motor2_Port |= _BV(M2X); else Motor2_Port &=~(_BV(M2X));
        if (Motor2 & 2) Motor2_Port |= _BV(M2Y); else Motor2_Port &=~(_BV(M2Y));
        singleShot(&Motor2,SENSE2);
        singleShot(&Motor1,SENSE1);
        if (OSCCAL != fcal){
            fcal=OSCCAL;
            eeprom_update_byte(&oscal,fcal);
            }
    }
}
