/*
 * Automated-World.c
 *
 * Created: 12/12/2018 1:54:37 PM
 * Author : Kiss My Keyboard Team
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include <stdlib.h>		/* Include standard library file */
#include <stdio.h>		/* Include standard I/O library file */
#include "MPU6050_res_define.h"							/* Include MPU6050 register define file */
#include "I2C_Master_H_file.h"							/* Include I2C Master header file */
#include "UART_H.h"										/* Include USART header file */
#include "MOTORS_Controller_H.h"						

#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define BIT(x) (0x01 << (x))

#define corner_1_x 0
#define corner_1_y 1
#define corner_2_x 2
#define corner_2_y 3
#define corner_3_x 4
#define corner_3_y 5
#define corner_4_x 6
#define corner_4_y 7


#define LEFT_IR_SENSOR_BIT 0
#define RIGHT_IR_SENSOR_BIT 4
#define LED_DISTANCE_BIT 5
#define LED_OUT_OF_BOX_BIT 1
#define RESET_POSITION_BUTTON_BIT 2

#define radians_factor 0.0174533
#define GYRO_THRESHOLD 0.3

#define LOWER_LIMIT 70
#define UPPER_LIMIT 100

void MPU_Start_Loc();
void MPU_Perform_Calc();
void MPU_Read_RawValue();
void Init_MPU6050();
void Init_UART();
void Init_PWM_Config();

void enable_distance_timer(bool true_val);
float get_X(float angle);
float get_Y(float angle);

void main_logic();
void serial_input_logic();
void lineTracker();
void Init_LineTracker_Config();
void reset_coordinates();
void config_debug_mode();
void check_switch_reset_coordinates();
void check_switch_debug_mode();
void Init_Buttons();
void Init_LEDS();


float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;
float pitch = 0;
float roll = 0;
float yaw = 0;
float v_pitch;
float v_roll;
float v_yaw;
float a_pitch;
float a_roll;
float a_yaw;
float Xg=0,Yg=0,Zg=0;
char buffer[20], float_[10];
float gyro_x_calc=0, gyro_y_calc=0, gyro_z_calc=0;


float current_x_pos = 0;
float current_y_pos = 0;
float past_x_pos = 0;
float past_y_pos = 0;

int num_of_boundaries_set;
long boundries[] = {0,0,0,0,0,0,0,0};

float toBeTraveled_X = 0;
float toBeTraveled_Y = 0;
float toBePosition_X = 0;
float toBePosition_Y = 0;

volatile float unit_direction = 1;
enum GAME_MODE {CALIBRATION, RUNTIME} current_mode;

volatile float travelled_distance;
	
int main(void)
{
	_delay_ms(1000);
	
	Init_Buttons();
	Init_LEDS();
	
	I2C_Init();											/* Initialize I2C */
	uart_init();										/* Initialize UART with 9600 baud rate */
	
	Init_MPU6050();										/* Initialize MPU6050 */	
	MOTORS_Init_Config();	
	stdout = &uart_output;
	stdin  = &uart_input;
	
	current_mode = CALIBRATION;
	
	num_of_boundaries_set = -1;
	
	sei();
	
	//current_mode = RUNTIME;
	//config_debug_mode();
	
	travelled_distance = 0;
	while(1) {		
		main_logic();
	}
	
	return 0;
}

void main_logic() {
	_delay_ms(20);
	if(current_mode == CALIBRATION) 
	{
		if(num_of_boundaries_set >= 4)
		{
			//num_of_boundaries_set = 0;
		}
		lineTracker();
		//check_switch_reset_coordinates();
		//check_switch_debug_mode();
	}
	else
	{	
		serial_input_logic();
			
	}
	
	// GYROSCOPE READINGS
	MPU_Read_RawValue();
	MPU_Perform_Calc();
	
	if(current_mode == CALIBRATION) {
		//* Phase 1: (Move down) till you hit a line with (Color Sensor), Set a limit (0,0)
		switch(num_of_boundaries_set) {
			case -1:
					if (Xg >= LOWER_LIMIT) {
						travelled_distance = 0;
						num_of_boundaries_set++;
						PORTB = PORTB^(1 << LED_OUT_OF_BOX_BIT);
						Xg = 0;
					}
					break;
			case 0:if((Xg >= LOWER_LIMIT) && (Xg < UPPER_LIMIT)) {
						boundries[corner_1_x] = 0;
						boundries[corner_1_y] = 0;
					} else if (Xg >= UPPER_LIMIT) {
						travelled_distance = 0;
						num_of_boundaries_set++;
						PORTB = PORTB^(1 << LED_OUT_OF_BOX_BIT);
						Xg = 0;
					}
					break;
			
			case 1:	if((Xg >= LOWER_LIMIT) && (Xg < UPPER_LIMIT)) {
						boundries[corner_2_x] = travelled_distance;
						boundries[corner_2_y] = 0;
					} else if (Xg >= UPPER_LIMIT) {
						travelled_distance = 0;
						num_of_boundaries_set++;
						Xg = 0;
						PORTB = PORTB^(1 << LED_OUT_OF_BOX_BIT);
					}
					break;
			
			case 2: if((Xg >= LOWER_LIMIT) && (Xg < UPPER_LIMIT)) {
						boundries[corner_3_x] = boundries[corner_2_x];
						boundries[corner_3_y] = travelled_distance;
					} else if (Xg >= UPPER_LIMIT) {
						travelled_distance = 0;
						num_of_boundaries_set++;
						PORTB = PORTB^(1 << LED_OUT_OF_BOX_BIT);

					}
					break;
					
			case 3: if((Xg >= LOWER_LIMIT) && (Xg < UPPER_LIMIT)) {
						boundries[corner_4_x] = 0;
						boundries[corner_4_y] = boundries[corner_3_y];
					} else if (Xg >= UPPER_LIMIT) {
						travelled_distance = 0;
						num_of_boundaries_set++;
						PORTB = PORTB^(1 << LED_OUT_OF_BOX_BIT);
					}
					break;
					
			case 4: current_mode = RUNTIME;
			//* Phase 6: Rotate car and move anywhere inside the closed rectangle. (Move by hand and press reset button)

		}	
		//printf("num_of_boundaries = %d \n", num_of_boundaries_set);
		
	} else {
		
		// Calculate (toBeTraveledDistance) relative to the PWM to be generated
		toBeTraveled_X = get_X(Xg);
		toBeTraveled_Y = get_Y(Xg);

		// Calculate (PositionToBeAt) using (CurrentPosition + toBeTraveledDistance)
		toBePosition_X = current_x_pos + (unit_direction*travelled_distance*toBeTraveled_X);
		toBePosition_Y = current_y_pos + (unit_direction*travelled_distance*toBeTraveled_Y);
		travelled_distance = 0;
		
		// if PositionToBeAt in &bounds:
		
		current_x_pos = toBePosition_X;
		current_y_pos = toBePosition_Y;
		
		/*
		dtostrf( current_x_pos, 3, 2, float_ );
		sprintf(buffer," Positions X = %s%c/s\t",float_,0xF8);
		printf(buffer);
		
		dtostrf( current_y_pos, 3, 2, float_ );
		sprintf(buffer," Position Y = %s%c/s\t",float_,0xF8);
		printf(buffer);
		*/
		// Using First Corner (Width) Only
		if (toBePosition_X < boundries[corner_2_x] && toBePosition_Y < boundries[corner_2_x] && toBePosition_X >= 0 && toBePosition_Y >= 0) 
		{	
			// Generate PWM accordingly and move car. (Already Generated)
			PORTB = (0 << LED_OUT_OF_BOX_BIT);

		}
		else 
		{
			// Stop Car and (flash lights accordingly)
			PORTB = (1 << LED_OUT_OF_BOX_BIT);
			
		}
	}
	
	/*
	dtostrf( Xg, 3, 2, float_ );
	sprintf(buffer," Gx = %s%c/s\t\n",float_,0xF8);
	printf(buffer);
	*/
	/*
	dtostrf( Xg, 3, 2, float_ );
	sprintf(buffer," Gx = %s%c/s\t",float_,0xF8);
	printf(buffer);
	
	dtostrf( Yg, 3, 2, float_ );
	sprintf(buffer," Gy = %s%c/s\t",float_,0xF8);
	printf(buffer);
	
	dtostrf( Zg, 3, 2, float_ );
	sprintf(buffer," Gz = %s%c/s\r\n",float_,0xF8);
	printf(buffer);
	
	*/
	_delay_ms(50);
	enable_distance_timer(false);
	MOTORS_stop();
	
}

void serial_input_logic()
{
	char input;
	input = getchar();

	if (input == 'F') {
		MOTORS_move_forward();
		enable_distance_timer(true);
		unit_direction = 1;
	} else if (input == 'B') {
		MOTORS_move_backward();
		enable_distance_timer(true);
		unit_direction = -1;
	} else if (input == 'R') {
		MOTORS_move_right();
	} else if (input == 'L') {
		MOTORS_move_left();
	} else if (input == 'S' ) {
		MOTORS_stop();
	} else if (input == 'W') {
		current_mode = RUNTIME;
	} else if (input == 'w') {
		current_mode = CALIBRATION;
	} else if (input == 'u' || input == 'U') {
		reset_coordinates();
	} else if(input == 'X') {
		reset_coordinates();
	} else if(input == 'Z') {
		reset_coordinates();
	} else {
		printf("Undefined Key!\n");
	}

}

void reset_coordinates() {
	Xg = 0;
	Yg = 0;
	Zg = 0;
	
	current_x_pos = 0;
	current_y_pos = 0;
	toBeTraveled_X = 0;
	toBeTraveled_Y = 0;
	toBePosition_X = 0;
	toBePosition_Y = 0;
}

void config_debug_mode() {
	boundries[corner_2_x] = 1000;
	boundries[corner_3_y] = 1000;
	current_mode = RUNTIME;	
}

void lineTracker(){
	if(bit_get(PINB , BIT(LEFT_IR_SENSOR_BIT)) && bit_get(PINB , BIT(RIGHT_IR_SENSOR_BIT))){
		//move(A , FORWARD , 255 - MOTOR_SPEED_HIGH);
		//move(B , FORWARD , 255 - MOTOR_SPEED_HIGH);
		_delay_ms(50);
		move(A , BACKWARD , 255 - MOTOR_SPEED_ROTATION_HI);
		move(B , FORWARD , MOTOR_SPEED_ROTATION_HI);
		_delay_ms(50);

		//printf("0\n");
	}
	else if (bit_get(PINB , BIT(LEFT_IR_SENSOR_BIT)) && ~(bit_get(PINB , BIT(RIGHT_IR_SENSOR_BIT)))){
		_delay_ms(50);
		move(A , BACKWARD , 255 - MOTOR_SPEED_ROTATION_HI);
		move(B , FORWARD ,MOTOR_SPEED_ROTATION_HI);
		_delay_ms(150);

		//printf("1\n");
	}
	else if (~(bit_get(PINB , BIT(LEFT_IR_SENSOR_BIT))) && bit_get(PINB , BIT(RIGHT_IR_SENSOR_BIT))){
		_delay_ms(50);
		move(A , FORWARD , MOTOR_SPEED_ROTATION_HI);
		move(B , BACKWARD , 255 - MOTOR_SPEED_ROTATION_HI);
		_delay_ms(150);
		//printf("2\n");
	}
	else if (~(bit_get(PINB , BIT(LEFT_IR_SENSOR_BIT))) && (~bit_get(PINB , BIT(RIGHT_IR_SENSOR_BIT)))){
		MOTORS_move_forward();
		_delay_ms(50);
		//printf("3\n");
	}
	enable_distance_timer(true);

}

void Init_LineTracker_Config(){
	// #PIN 14 : Left Sensor [PIN 8 in Arduino (PB0)
	// #PIN 18 : Right Sensor [PIN 12 in Arduino (PB4)
	
	DDRB &= 0b11101110;
	
}

void check_switch_reset_coordinates() {
	if(bit_get(DDRB, BIT(RESET_POSITION_BUTTON_BIT))) {
		_delay_ms(50);
		reset_coordinates();
	} 
}

void check_switch_debug_mode() {
	if(bit_get(DDRB, BIT(RESET_POSITION_BUTTON_BIT))) {
		_delay_ms(50);
		config_debug_mode();
		_delay_ms(50);
	}
}

void Init_Buttons() {
	bit_clear(DDRB, BIT(RESET_POSITION_BUTTON_BIT));
}

void Init_LEDS() {
	bit_set(DDRB, BIT(LED_DISTANCE_BIT));
	bit_set(DDRB, BIT(LED_OUT_OF_BOX_BIT));
}

/************************************************************************/
/* Distance Timer ISR, config, and helper methods                       */
/************************************************************************/
ISR (TIMER1_OVF_vect, ISR_NAKED)
{
	TCNT1H = 0xB1;
	TCNT1L = 0xE0;
	
	travelled_distance++;
	
	PORTB = PORTB^(1 << LED_DISTANCE_BIT);

	reti();
}

void enable_distance_timer(bool true_val) {
	if(true_val) {
		sei();
		// Timer of 10 miliseconds
		// Prescalar = 8
		// Overflow Interrupt Mode
		TCNT1H = 0xB1;
		TCNT1L = 0xE0;
		TCCR1A = 0x00;
		TCCR1B = 0b00000010;
		TIMSK1 = (1 << TOIE1);
	} else {
		// Resetting the timer and stopping the clk
		TCNT1H = 0xB1;
		TCNT1L = 0xE0;
		TCCR1A = 0x00;
		TCCR1B = 0b00000000;
		TIMSK1 = (1 << TOIE1);
	}
}

float get_X(float angle)
{
	return (cos(angle*radians_factor));
}

float get_Y(float angle)
{
	return (sin(angle*radians_factor));

}


/************************************************************************/
/*  GYROSCOPE Worker Methods                                            */
/************************************************************************/
void MPU_Start_Loc()
{
	I2C_Start_Wait(0xD0);								/* I2C start with device write address */
	I2C_Write(ACCEL_XOUT_H);							/* Write start location address from where to read */
	I2C_Repeated_Start(0xD1);							/* I2C start with device read address */
}

void MPU_Perform_Calc()
{
	// Calculating Pitch, roll and yaw
	v_pitch=(Gyro_x/131);
	if(v_pitch==-1)
	v_pitch=0;
	v_roll=(Gyro_y/131);
	if(v_roll==1)
	v_roll=0;
	v_yaw=Gyro_z/131;
	a_pitch=(v_pitch*0.046);
	a_roll=(v_roll*0.046);
	a_yaw=(v_yaw*0.045);
	pitch= pitch + a_pitch;
	roll= roll + a_roll;
	yaw= yaw + a_yaw;
	
	gyro_x_calc = Gyro_x/131;
	gyro_y_calc = Gyro_y/131;
	gyro_z_calc = Gyro_z/131;
	
	if(((gyro_x_calc +0.078) > GYRO_THRESHOLD) || ((gyro_x_calc +0.078) < -GYRO_THRESHOLD)) {
		Xg += (gyro_x_calc/1.25) +0.078;
		if(Xg >= 359) {
			Xg-= 360;
		}
		if(Xg <= -359) {
			Xg+= 360;
		}
	}
	
	if((gyro_y_calc > GYRO_THRESHOLD) || (gyro_y_calc < -GYRO_THRESHOLD)) {
		Yg += gyro_y_calc;
		if(Yg >= 359) {
			Yg-= 360;
		}
		if(Yg <= -359) {
			Yg+= 360;
		}
	}
	
	if((gyro_z_calc > GYRO_THRESHOLD) || (gyro_z_calc < -GYRO_THRESHOLD)) {
		Zg += gyro_z_calc;
		if(Zg >= 359) {
			Zg-= 360;
		}
		if(Zg <= -359) {
			Zg+= 360;
		}
	}
}

void MPU_Read_RawValue()
{
	MPU_Start_Loc();									/* Read Gyro values */
	Acc_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Acc_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Temperature = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_x = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_y = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Ack());
	Gyro_z = (((int)I2C_Read_Ack()<<8) | (int)I2C_Read_Nack());
	I2C_Stop();
}

void Init_MPU6050()										/* Gyro initialization function */
{
	_delay_ms(150);										/* Power up time >100ms */
	I2C_Start_Wait(0xD0);								/* Start with device write address */
	I2C_Write(SMPLRT_DIV);								/* Write to sample rate register */
	I2C_Write(0x07);									/* 1KHz sample rate */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(PWR_MGMT_1);								/* Write to power management register */
	I2C_Write(0x01);									/* X axis gyroscope reference frequency */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(CONFIG);									/* Write to Configuration register */
	I2C_Write(0x00);									/* Fs = 8KHz */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(GYRO_CONFIG);								/* Write to Gyro configuration register */
	I2C_Write(0x18);									/* Full scale range +/- 2000 degree/C */
	I2C_Stop();

	I2C_Start_Wait(0xD0);
	I2C_Write(INT_ENABLE);								/* Write to interrupt enable register */
	I2C_Write(0x01);
	I2C_Stop();
}
