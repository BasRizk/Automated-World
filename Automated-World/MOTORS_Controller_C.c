/*
 * MOTORS_Controller_C.c
 *
 * Created: 12/14/2018 9:44:20 PM
 *  Author: Kiss My Keyboard
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "MOTORS_Controller_H.h"

#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define BIT(x) (0x01 << (x))

	
void MOTORS_Init_PWM_Config() {
	// #PIN 2: A-1B #PIN 6: A-1A
	// #PIN 4: B-1B #PIN 5: B-1A
	
	DDRD |= 0b01110100;
	
	_delay_ms(150);
	OCR0A = 0;
	OCR0B = 0;
	
	OCR0A = 102;
	OCR0B = 102;
	bit_set(PORTD, BIT(A_DIR_BIT));
	bit_set(PORTD, BIT(B_DIR_BIT));
	TCCR0A = 0b10100001;
	TCCR0B = 0b00000001;
}

void MOTORS_Init_Config() {
	MOTORS_Init_PWM_Config();
	move(A, BACKWARD, 0);
	move(B, BACKWARD, 0);
}

void move(enum MOTOR motor, enum DIRECTION direction, unsigned char speed) {
	switch(motor) {
		case A:	if(direction == BACKWARD) {
			bit_clear(PORTD, BIT(A_DIR_BIT));
			} else {
			bit_set(PORTD, BIT(A_DIR_BIT));
		}
		OCR0B = speed;
		break;
		
		case B:	if(direction == BACKWARD) {
			bit_clear(PORTD, BIT(B_DIR_BIT));
			} else {
			bit_set(PORTD, BIT(B_DIR_BIT));
		}
		OCR0A = speed;
		break;
	}
}

void MOTORS_move_forward() {
	move(A, FORWARD, 256 - MOTOR_SPEED_NORMAL_HI);
	move(B, FORWARD, 256 - MOTOR_SPEED_NORMAL_HI);
}

void MOTORS_move_backward() {
	move(A, BACKWARD, MOTOR_SPEED_NORMAL_HI);
	move(B, BACKWARD, MOTOR_SPEED_NORMAL_HI);
}

void MOTORS_move_left() {
	move(A, BACKWARD, 256 - MOTOR_SPEED_ROTATION_HI);
	move(B, FORWARD, MOTOR_SPEED_ROTATION_HI);
}

void MOTORS_move_right() {
	move(A, FORWARD, MOTOR_SPEED_ROTATION_HI);
	move(B, BACKWARD, 256 - MOTOR_SPEED_ROTATION_HI);
}

void MOTORS_stop() {
	move(A, BACKWARD, 0);
	move(B, BACKWARD, 0);
}