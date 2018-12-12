/*
 * Automated-World.c
 *
 * Created: 12/12/2018 1:54:37 PM
 * Author : mikel
 */ 

#include <avr/io.h>
#include <stdbool.h>

#define corner_1_x 0
#define corner_1_y 1
#define corner_2_x 2
#define corner_2_y 3
#define corner_3_x 4
#define corner_3_y 5
#define corner_4_x 6
#define corner_4_y 7


int main(void)
{
	bool calibration = true;
	float current_x_pos = 0;
	float current_y_pos = 0;
	float past_x_pos = 0;
	float past_y_pos = 0;
	float boundries[] = {0,0,0,0,0,0,0,0};
	
	if(calibration) {		// Is (Calibration) Mode or (Running) Mode?
		
		// wait for button press, if clicked:
		while(1 /* !button? */) {}
		
		// car is moving and scanning the borders, limits should be set according to Cartesian coordinates:
		
			//* Phase 1: (Move down) till you hit a line with (Color Sensor), Set a limit (0,0)
		while(1 /* !color sensor */) {
			// move car down
		}
		
			//* Once hit a line, (Move counter Clockwise) accordingly with the line being tracked.
		while(1 /* !threshold */) {
			// move car right
		}
		
			//* Phase 2: Once hit a threshold of (y-change) while moving, Push a limit (current_X, 0).
		boundries[corner_1_x] = current_x_pos;
		boundries[corner_1_y] = 0;
		while(1 /* !threshold */) {
			// move car up
		}
		
			//* Phase 3: Once hit a threshold of (x-change) while moving, Push a limit (past_limit_X, current_Y)
		boundries[corner_2_x] = past_x_pos;
		boundries[corner_2_y] = current_y_pos;
		while(1 /* !threshold */) {
			// move car left
		}
		
			//* Phase 4: Once hit a threshold of (y-change) while moving, Push a limit (current_X, past_limit_Y)
		boundries[corner_3_x] = current_x_pos;
		boundries[corner_3_y] = past_y_pos;	
		
			//* Phase 5: Push a limit (last_X, 0)
		boundries[corner_4_x] = past_x_pos;
		boundries[corner_4_y] = 0;
			
			//* Phase 6: Rotate car and move anywhere inside the closed rectangle.


			
		calibration = 0;		// After calibration -> turn to (Running) Mode.
	}
	else if(!calibration) {	// else if (Running) Mode:
		// wait for button press, if clicked:
			// Calculate (toBeTraveledDistance) relative to the PWM to be generated
			// Calculate (PositionToBeAt) using (CurrentPosition + toBeTraveledDistance)
			
			if (1) {		// Is PositionToBeAt in &bounds or Not:
				// Calculate &bounds using (limits[4] array] -- Could be done previously
		
				if (1) {		// if PositionToBeAt in &bounds:
					// Generate PWM accordingly and move car.
				}
				else {
					// Stop Car and flash lights accordingly
				}
			}
	}
	
	return 0;
}
