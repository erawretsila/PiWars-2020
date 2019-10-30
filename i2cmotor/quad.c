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
 * quadrature decode module for Z-Gun interface board
 * Quad 1 inputs on D0 & D1
 * Quad 2 Inputs of D2 & D6
 *  counts stored as 16 bit ints & copied to output registers on 
 * timer A overflow to ensure atomicity
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "quad.h"
volatile uint16_t rtquada=0;
volatile uint16_t rtquadb=0;
volatile uint8_t qstatea=0;
volatile uint8_t qstateb=0;

uint8_t changed;

ISR(PCINT2_vect){
    uint8_t temp=PIND;
    changed=pstate ^ temp;
    if ((changed & _BV(0)) | (changed & _BV(1))){ //quadrature A
        if ((pstate & _BV(0)>>0)== ((temp & _BV(1))>>1)) { //Prev a & b equal
            rtquada++;
        } else {                //prev in 1 7 current input 2 different
            rtquada--;

        } 
 
    }
    if ((changed & _BV(2)) | (changed & _BV(6))){ //quadrature A

        if ((pstate & _BV(2))== ((temp & _BV(6))>>4)) { //Prev a & b equal
            rtquadb++;
        } else {                //prev in 1 7 current input 2 different
            rtquadb--;

        } 
 
    }
    pstate=temp;
}
    


void QuadInit(void){
    PCMSK2=_BV(PCINT11)|_BV(PCINT12)|_BV(PCINT13)|_BV(PCINT17);
    DDRD &= ~(_BV(PB0)|_BV(PB2)|_BV(PB3)|_BV(PB4));
    GIMSK|=_BV(PCIE2);
    DDRB |=_BV(4);
    pstate=PIND;
    PORTB |=_BV(04);
}


