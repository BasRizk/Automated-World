/*
 * Automated-World.c
 *
 * Created: 12/12/2018 1:54:37 PM
 * Author : mikel
 */ 

#include <avr/io.h>

typedef int bool;
#define true 1
#define false 0

int main(void)
{
	bool calibration = true;
	
	if(calibration) {		// Is (Calibration) Mode or (Running) Mode?
		// wait for button press, if clicked:
		// car is moving and scanning the borders, limits should be set according to Cartesian coordinates:
			//* Phase 1: (Move down) till you hit a line with (Color Sensor), Set a limit (0,0)
			//* Once hit a line, (Move counter Clockwise) accordingly with the line being tracked.
			//* Phase 2: Once hit a threshold of (y-change) while moving, Push a limit (current_X, 0).
			//* Phase 3: Once hit a threshold of (x-change) while moving, Push a limit (past_limit_X, current_Y)
			//* Phase 4: Once hit a threshold of (y-change) while moving, Push a limit (current_X, past_limit_Y)
			//* Phase 5: Push a limit (last_X, 0)
			//* Phase 6: Rotate car and move anywhere inside the closed rectangle.
			
		//calibration = 0;		// After calibration -> turn to (Running) Mode.
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
