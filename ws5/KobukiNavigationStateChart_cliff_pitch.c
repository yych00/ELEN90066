/*
* KobukiNavigationStateChart_cliff_easy_simulation_pitch.c
*
*/

#include "KobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>


// Program States
typedef enum{
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	APPROACH_RAMP,						// Drive on level ground until the uphill ramp is reached
	CLIMB_RAMP,							// Drive up the ramp
	DRIVE_ACROSS_TOP,					// Drive along the level road at the top
	DESCEND_RAMP,						// Drive down the ramp
	DRIVE_ON_LEVEL,						// Continue straight after descending to level ground
	CLIFF_BACKUP,							// Reverse away from a detected cliff
	CLIFF_TURN_RIGHT,						// Turn right after reversing
	CLIFF_TURN_LEFT						// Turn left after reversing

} robotState_t;

/* accelAxes is measured in g; pitch is measured in degrees.
 * Positive pitch means uphill. Use +/-5 degrees to classify slopes;
 * smaller tilt magnitudes are treated as level ground.
 */
#define PITCH_THRESHOLD_DEG		5.0
#define RAD_TO_DEG				57.29577951308232
/* Exponential smoothing: smaller alpha is smoother but responds slower. */
#define PITCH_FILTER_ALPHA       0.20
#define STABLE_SAMPLE_COUNT		15

#define APPROACH_SPEED_MM_S		160
#define CLIMB_SPEED_MM_S		200
#define TOP_SPEED_MM_S			100
#define DESCEND_SPEED_MM_S		80
#define CLIFF_TURN_SPEED_MM_S	80
#define CLIFF_BACKUP_FAST_MM_S	80
#define CLIFF_BACKUP_SLOW_MM_S	50
#define CLIFF_BACKUP_DISTANCE_MM	80
#define CLIFF_TURN_ANGLE_DEG		30

/* Keep the lateral acceleration near zero while travelling on a slope. */
#define Y_ALIGNMENT_THRESHOLD_G	0.008
#define Y_FILTER_SAMPLE_COUNT    5
#define UPHILL_ALIGN_OUTER_SPEED_MM_S	140
#define UPHILL_ALIGN_INNER_SPEED_MM_S	120
#define DOWNHILL_ALIGN_OUTER_SPEED_MM_S 70
#define DOWNHILL_ALIGN_INNER_SPEED_MM_S 50

/* Fixed wheel calibration, independent of slope alignment.
 * -0.01: boost the left wheel by 1% to correct a leftward drift.
 * +0.01: boost the right wheel by 1% to correct a rightward drift.
 *  0.00: no boost. Edit this value and rebuild to tune the robot.
 */
#ifndef WHEEL_SPEED_TRIM
#define WHEEL_SPEED_TRIM         0.00
#endif

static int16_t trimWheelSpeed(const int16_t speed, const double boost,
                             const int16_t maxWheelSpeed)
{
	/* Preserve direction, round to mm/s, then clamp before the int16 cast. */
	double magnitude = floor(fabs((double)speed) * (1.0 + boost) + 0.5);
	const double speedLimit = fabs((double)maxWheelSpeed);
	if (magnitude > speedLimit){
		magnitude = speedLimit;
	}
	if (magnitude > 32767.0){
		magnitude = 32767.0;
	}
	return (int16_t)((speed < 0) ? -magnitude : magnitude);
}

static int16_t limitSpeed(const int16_t requestedSpeed, const int16_t maxWheelSpeed)
{
	int16_t positiveLimit = maxWheelSpeed;

	if (positiveLimit < 0){
		positiveLimit = (int16_t)-positiveLimit;
	}
	return (requestedSpeed < positiveLimit) ? requestedSpeed : positiveLimit;
}

void KobukiNavigationStatechart(
	const int16_t 				maxWheelSpeed,
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const KobukiSensors_t		sensors,
	const accelerometer_t		accelAxes,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed,
	const bool					isSimulator
	){

	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = APPROACH_RAMP;	// state history for pause region
	static robotState_t			stateBeforeCliff = APPROACH_RAMP;
	static int16_t				stableSampleCounter = 0;
	static int16_t				downhillSampleCounter = 0;
	static int32_t				cliffStartDistance = 0;
	static int32_t				cliffStartAngle = 0;
	static bool					cliffTurnRight = true;
	static double                   filteredPitchDeg = 0.0;
	static bool                     pitchFilterInitialized = false;
	static double                   ySamples[Y_FILTER_SAMPLE_COUNT] = {0};
	static double                   ySampleSum = 0.0;
	static int                      ySampleIndex = 0;
	static int                      ySampleCount = 0;

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************
	/* X points along the robot; Y and Z form the transverse plane.
	 * This accelerometer-only estimate assumes gravity dominates motion.
	 */
	const double transverseGravity = sqrt(
		(double)accelAxes.y * accelAxes.y
		+ (double)accelAxes.z * accelAxes.z);
	const double rawPitchDeg = atan2((double)accelAxes.x, transverseGravity)
		* RAD_TO_DEG;
	/* Seed from the first sample; do not introduce a fictitious zero tilt.
	 * Update on every call, including pause and cliff avoidance.
	 */
	if (!pitchFilterInitialized){
		filteredPitchDeg = rawPitchDeg;
		pitchFilterInitialized = true;
	}
	else{
		filteredPitchDeg += PITCH_FILTER_ALPHA * (rawPitchDeg - filteredPitchDeg);
	}
	const double pitchDeg = filteredPitchDeg;
	const bool uphillSlope = (pitchDeg >= PITCH_THRESHOLD_DEG);
	const bool downhillSlope = (pitchDeg <= -PITCH_THRESHOLD_DEG);
	const bool onSlope = uphillSlope || downhillSlope;
	/* Rolling mean of the latest five raw Y readings, updated every call.
	 * Wait for a complete window after startup/reset before correcting.
	 */
	if (ySampleCount == Y_FILTER_SAMPLE_COUNT){
		ySampleSum -= ySamples[ySampleIndex];
	}
	else{
		ySampleCount++;
	}
	ySamples[ySampleIndex] = accelAxes.y;
	ySampleSum += accelAxes.y;
	ySampleIndex = (ySampleIndex + 1) % Y_FILTER_SAMPLE_COUNT;
	const bool yFilterReady = (ySampleCount == Y_FILTER_SAMPLE_COUNT);
	const double filteredY = ySampleSum / ySampleCount;
	const bool cliffDetected = sensors.cliffLeft
		|| sensors.cliffCenter
		|| sensors.cliffRight;


	/* B1 is a reset button: stop and require a fresh B0 press to start. */
	if (sensors.buttons.B1){
		ySampleSum = 0.0;
		ySampleIndex = 0;
		ySampleCount = 0;
		pitchFilterInitialized = false;
		state = UNPAUSE_WAIT_BUTTON_PRESS;
		unpausedState = APPROACH_RAMP;
		stableSampleCounter = 0;
		downhillSampleCounter = 0;
	}
	else if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0				// pause button
		){
		stableSampleCounter = 0;
		downhillSampleCounter = 0;
		switch (state){
		case INITIAL:
			ySampleSum = 0.0;
			ySampleIndex = 0;
			ySampleCount = 0;
			pitchFilterInitialized = false;
			/* Start every run from the beginning of the straight course. */
			unpausedState = APPROACH_RAMP;
			stableSampleCounter = 0;
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// state transition - run region      *
	//*************************************
	else if (cliffDetected
		&& (state == APPROACH_RAMP
			|| state == CLIMB_RAMP
			|| state == DRIVE_ACROSS_TOP
			|| state == DESCEND_RAMP
			|| state == DRIVE_ON_LEVEL)){
		/* Save the route state, then back away before turning. */
		stateBeforeCliff = state;
		cliffTurnRight = sensors.cliffLeft || sensors.cliffCenter;
		cliffStartDistance = netDistance;
		stableSampleCounter = 0;
		downhillSampleCounter = 0;
		state = CLIFF_BACKUP;
	}
	else if (state == CLIFF_BACKUP
		&& abs(netDistance - cliffStartDistance) >= CLIFF_BACKUP_DISTANCE_MM){
		cliffStartAngle = netAngle;
		state = cliffTurnRight ? CLIFF_TURN_RIGHT : CLIFF_TURN_LEFT;
	}
	else if ((state == CLIFF_TURN_RIGHT || state == CLIFF_TURN_LEFT)
		&& abs(netAngle - cliffStartAngle) >= CLIFF_TURN_ANGLE_DEG){
		state = stateBeforeCliff;
	}
	else if (downhillSlope
		&& state != DESCEND_RAMP
		&& state != CLIFF_BACKUP
		&& state != CLIFF_TURN_RIGHT
		&& state != CLIFF_TURN_LEFT){
		/* A stable negative pitch means the robot is facing downhill. */
		stableSampleCounter = 0;
		if (downhillSampleCounter < STABLE_SAMPLE_COUNT){
			downhillSampleCounter++;
		}
		if (downhillSampleCounter >= STABLE_SAMPLE_COUNT){
			state = DESCEND_RAMP;
			downhillSampleCounter = 0;
		}
	}
	else{
		bool transitionCondition = false;
		robotState_t nextState = state;
		downhillSampleCounter = 0;

		switch (state){
		case APPROACH_RAMP:
			transitionCondition = uphillSlope;
			nextState = CLIMB_RAMP;
			break;
		case CLIMB_RAMP:
			transitionCondition = !onSlope;
			nextState = DRIVE_ACROSS_TOP;
			break;
		case DRIVE_ACROSS_TOP:
			transitionCondition = downhillSlope;
			nextState = DESCEND_RAMP;
			break;
		case DESCEND_RAMP:
			transitionCondition = !onSlope;
			nextState = DRIVE_ON_LEVEL;
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
	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch (state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
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

	case DRIVE_ON_LEVEL:
		leftWheelSpeed = rightWheelSpeed = limitSpeed(APPROACH_SPEED_MM_S, maxWheelSpeed);
		break;

	case CLIFF_BACKUP:
		if (cliffTurnRight){
			/* Reverse in a right-hand arc, away from a left/centre cliff. */
			leftWheelSpeed = -limitSpeed(CLIFF_BACKUP_SLOW_MM_S, maxWheelSpeed);
			rightWheelSpeed = -limitSpeed(CLIFF_BACKUP_FAST_MM_S, maxWheelSpeed);
		}
		else{
			/* Reverse in a left-hand arc, away from a right cliff. */
			leftWheelSpeed = -limitSpeed(CLIFF_BACKUP_FAST_MM_S, maxWheelSpeed);
			rightWheelSpeed = -limitSpeed(CLIFF_BACKUP_SLOW_MM_S, maxWheelSpeed);
		}
		break;

	case CLIFF_TURN_RIGHT:
		leftWheelSpeed = limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
		rightWheelSpeed = -limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
		break;

	case CLIFF_TURN_LEFT:
		leftWheelSpeed = -limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
		rightWheelSpeed = limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}

	/*
	 * Y = 0 has two possible headings: straight uphill or straight downhill.
	 * The correction direction must therefore be reversed while descending.
	 * Check the five-sample Y mean on every call during climb/descent,
	 * without pitch gating or an additional confirmation counter.
	 * Returning to the mean's deadband restores normal straight speed.
	 * Do not apply slope alignment on level-road states, where acceleration
	 * spikes could otherwise make the robot turn back toward the ramp.
	 */
	if (state == CLIMB_RAMP && yFilterReady){
		if (filteredY >= Y_ALIGNMENT_THRESHOLD_G){
			/* Uphill, positive Y: curve right. */
			leftWheelSpeed = limitSpeed(UPHILL_ALIGN_OUTER_SPEED_MM_S, maxWheelSpeed);
			rightWheelSpeed = limitSpeed(UPHILL_ALIGN_INNER_SPEED_MM_S, maxWheelSpeed);
		}
		else if (filteredY <= -Y_ALIGNMENT_THRESHOLD_G){
			/* Uphill, negative Y: curve left. */
			leftWheelSpeed = limitSpeed(UPHILL_ALIGN_INNER_SPEED_MM_S, maxWheelSpeed);
			rightWheelSpeed = limitSpeed(UPHILL_ALIGN_OUTER_SPEED_MM_S, maxWheelSpeed);
		}
	}
	else if (state == DESCEND_RAMP && yFilterReady){
		if (filteredY >= Y_ALIGNMENT_THRESHOLD_G){
			/* Downhill, positive Y: curve left (opposite to uphill). */
			leftWheelSpeed = limitSpeed(DOWNHILL_ALIGN_INNER_SPEED_MM_S, maxWheelSpeed);
			rightWheelSpeed = limitSpeed(DOWNHILL_ALIGN_OUTER_SPEED_MM_S, maxWheelSpeed);
		}
		else if (filteredY <= -Y_ALIGNMENT_THRESHOLD_G){
			/* Downhill, negative Y: curve right (opposite to uphill). */
			leftWheelSpeed = limitSpeed(DOWNHILL_ALIGN_OUTER_SPEED_MM_S, maxWheelSpeed);
			rightWheelSpeed = limitSpeed(DOWNHILL_ALIGN_INNER_SPEED_MM_S, maxWheelSpeed);
		}
	}


	/* Apply calibration to final commands, including reverse and turns.
	 * A stopped wheel stays stopped; saturation can reduce the boost.
	 */
	*pLeftWheelSpeed = trimWheelSpeed(leftWheelSpeed,
		(WHEEL_SPEED_TRIM < 0.0) ? -WHEEL_SPEED_TRIM : 0.0, maxWheelSpeed);
	*pRightWheelSpeed = trimWheelSpeed(rightWheelSpeed,
		(WHEEL_SPEED_TRIM > 0.0) ? WHEEL_SPEED_TRIM : 0.0, maxWheelSpeed);

	(void)isSimulator;
}
