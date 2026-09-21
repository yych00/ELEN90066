/*
* KobukiNavigationStatechart.c
*
*/

#include "KobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

typedef enum{
	INITIAL = 0,
	PAUSE_WAIT_BUTTON_RELEASE,
	UNPAUSE_WAIT_BUTTON_PRESS,
	UNPAUSE_WAIT_BUTTON_RELEASE,
	APPROACH_RAMP,
	CLIMB_RAMP,
	DRIVE_ACROSS_TOP,
	DESCEND_RAMP,
	FINISHED
} robotState_t;

#define SLOPE_THRESHOLD_G		0.10
#define STABLE_SAMPLE_COUNT		5

#define APPROACH_SPEED_MM_S		100
#define CLIMB_SPEED_MM_S		150
#define TOP_SPEED_MM_S			100
#define DESCEND_SPEED_MM_S		80
#define CLIFF_TURN_SPEED_MM_S	80

static int16_t limitSpeed(const int16_t requestedSpeed, const int16_t maxWheelSpeed)
{
	int16_t positiveLimit = maxWheelSpeed;

	if (positiveLimit < 0){
		positiveLimit = (int16_t)-positiveLimit;
	}
	return (requestedSpeed < positiveLimit) ? requestedSpeed : positiveLimit;
}

void KobukiNavigationStatechart(
	const int16_t maxWheelSpeed,
	const int32_t netDistance,
	const int32_t netAngle,
	const KobukiSensors_t sensors,
	const accelerometer_t accelAxes,
	int16_t * const pRightWheelSpeed,
	int16_t * const pLeftWheelSpeed,
	const bool isSimulator
	){

	static robotState_t state = INITIAL;
	static robotState_t unpausedState = APPROACH_RAMP;
	static int16_t stableSampleCounter = 0;

	int16_t leftWheelSpeed = 0;
	int16_t rightWheelSpeed = 0;

	const bool onSlope = (fabs(accelAxes.x) >= SLOPE_THRESHOLD_G);
	const bool cliffDetected = sensors.cliffLeft
		|| sensors.cliffCenter
		|| sensors.cliffRight;

	/* B1 is a reset button: stop and require a fresh B0 press to start. */
	if (sensors.buttons.B1){
		state = UNPAUSE_WAIT_BUTTON_PRESS;
		unpausedState = APPROACH_RAMP;
		stableSampleCounter = 0;
	}
	else if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0){
		switch (state){
		case INITIAL:
			unpausedState = APPROACH_RAMP;
			stableSampleCounter = 0;
			state = UNPAUSE_WAIT_BUTTON_PRESS;
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			if (!sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			if (!sensors.buttons.B0){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			if (sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	else if (cliffDetected){
		/* Keep the current hill state while steering away from the cliff. */
		stableSampleCounter = 0;
	}
	else{
		bool transitionCondition = false;
		robotState_t nextState = state;

		switch (state){
		case APPROACH_RAMP:
			transitionCondition = onSlope;
			nextState = CLIMB_RAMP;
			break;
		case CLIMB_RAMP:
			transitionCondition = !onSlope;
			nextState = DRIVE_ACROSS_TOP;
			break;
		case DRIVE_ACROSS_TOP:
			transitionCondition = onSlope;
			nextState = DESCEND_RAMP;
			break;
		case DESCEND_RAMP:
			transitionCondition = !onSlope;
			nextState = FINISHED;
			break;
		default:
			transitionCondition = false;
			break;
		}

		if (transitionCondition){
			if (stableSampleCounter < STABLE_SAMPLE_COUNT){
				stableSampleCounter++;
			}
			if (stableSampleCounter >= STABLE_SAMPLE_COUNT){
				state = nextState;
				stableSampleCounter = 0;
			}
		}
		else{
			stableSampleCounter = 0;
		}
	}

	switch (state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	case APPROACH_RAMP:
		leftWheelSpeed = rightWheelSpeed = limitSpeed(APPROACH_SPEED_MM_S, maxWheelSpeed);
		break;
	case CLIMB_RAMP:
		leftWheelSpeed = rightWheelSpeed = limitSpeed(CLIMB_SPEED_MM_S, maxWheelSpeed);
		break;
	case DRIVE_ACROSS_TOP:
		leftWheelSpeed = rightWheelSpeed = limitSpeed(TOP_SPEED_MM_S, maxWheelSpeed);
		break;
	case DESCEND_RAMP:
		leftWheelSpeed = rightWheelSpeed = limitSpeed(DESCEND_SPEED_MM_S, maxWheelSpeed);
		break;
	case FINISHED:
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	default:
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}

	if (state == APPROACH_RAMP
		|| state == CLIMB_RAMP
		|| state == DRIVE_ACROSS_TOP
		|| state == DESCEND_RAMP){
		if (sensors.cliffLeft || sensors.cliffCenter){
			leftWheelSpeed = limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
			rightWheelSpeed = -limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
		}
		else if (sensors.cliffRight){
			leftWheelSpeed = -limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
			rightWheelSpeed = limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
		}
	}

	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;

	(void)netDistance;
	(void)netAngle;
	(void)isSimulator;
}
