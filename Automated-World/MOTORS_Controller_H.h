/*
 * MOTORS_Controller_H.h
 *
 * Created: 12/14/2018 9:57:08 PM
 *  Author: Xbass
 */ 


#ifndef MOTORS_CONTROLLER_H_H_
#define MOTORS_CONTROLLER_H_H_

#define MOTOR_SPEED_NORMAL_MED 102
#define MOTOR_SPEED_NORMAL_HI 180
#define MOTOR_SPEED_ROTATION_HI 230

enum MOTOR {A, B};
enum DIRECTION {BACKWARD, FORWARD};

// BITS MACROS
#define A_DIR_BIT 2
#define B_DIR_BIT 4

void MOTORS_Init_Config();
void MOTORS_move_forward();
void MOTORS_move_backward();
void MOTORS_move_left();
void MOTORS_move_right();
void MOTORS_stop();
void move(enum MOTOR, enum DIRECTION, unsigned char speed);

#endif /* MOTORS_CONTROLLER_H_H_ */