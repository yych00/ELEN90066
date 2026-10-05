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

/* 【坡度控制：前后倾角判断】加速度单位为 g，pitch 单位为度。
 * pitch >= +5° 为上坡，pitch <= -5° 为下坡，中间按平地处理。
 * 最近 8 次原始 pitch 中至少 7 次满足条件才切换状态；允许 1 次不满足。
 * 启动、暂停、避让及状态切换后重新收满 8 次；不对 pitch 取平均。
 */
#define PITCH_THRESHOLD_DEG		5.0                 // 上下坡角度阈值
#define RAD_TO_DEG				20    				// 弧度转角度：180 / pi
#define PITCH_WINDOW_SAMPLE_COUNT 8                 // pitch 检测滑动窗口
#define PITCH_REQUIRED_SAMPLE_COUNT 7               // 窗口内至少 7 次同类结果

/* 【坡度控制：各行驶阶段的基础轮速】单位：mm/s。 */
#define APPROACH_SPEED_MM_S		150 // 接近坡道、下坡后平地直行
#define CLIMB_SPEED_MM_S		200 // 上坡正常直行
#define TOP_SPEED_MM_S			150 // 坡顶平地直行
#define DESCEND_SPEED_MM_S		100  // 下坡正常直行

/* 【Cliff 悬崖避让：先弧线倒车，再原地转向】不使用 pitch 或 Y 判断。 */
#define CLIFF_TURN_SPEED_MM_S	80  // 原地转向轮速大小，左右轮方向相反
#define CLIFF_BACKUP_FAST_MM_S	80  // 倒车较快轮速度大小，输出时加负号
#define CLIFF_BACKUP_SLOW_MM_S	50  // 倒车较慢轮速度大小，输出时加负号
#define CLIFF_BACKUP_DISTANCE_MM	60  // 倒车距离阈值：60 mm
#define CLIFF_TURN_ANGLE_DEG		30  // 避让转向角度：30°

/* 【坡度控制：Y 方向差速纠偏】只在上坡、下坡状态中生效。
 * 最近 5 次 Y 取平均，与 Y_ALIGNMENT_THRESHOLD_G 的正负阈值比较；不额外连续确认。
 * 外侧轮比内侧轮快，左右轮的分配由纠偏方向决定。
 */
#define Y_ALIGNMENT_THRESHOLD_G	0.01 // Y 平均值的纠偏阈值，单位 g
#define Y_FILTER_SAMPLE_COUNT    5   // Y 滑动平均窗口，区别于 pitch 的 8 次中 7 次检测
#define UPHILL_ALIGN_OUTER_SPEED_MM_S	200 // 上坡纠偏外侧轮
#define UPHILL_ALIGN_INNER_SPEED_MM_S	150 // 上坡纠偏内侧轮
#define DOWNHILL_ALIGN_OUTER_SPEED_MM_S 100 // 下坡纠偏外侧轮
#define DOWNHILL_ALIGN_INNER_SPEED_MM_S 75 // 下坡纠偏内侧轮

/* 【通用轮速补偿：坡度行驶与 Cliff 避让都应用】
 * Fixed wheel calibration, independent of slope alignment.
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
	static int                      pitchSamples[PITCH_WINDOW_SAMPLE_COUNT] = {0};
	static int                      pitchSampleIndex = 0;
	static int                      pitchSampleCount = 0;
	static int32_t				cliffStartDistance = 0;
	static int32_t				cliffStartAngle = 0;
	static bool					cliffTurnRight = true;
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
	/* 【坡度控制】使用加速度计计算原始 pitch，供上下坡状态判断。
	 * X points along the robot; Y and Z form the transverse plane.
	 * This accelerometer-only estimate assumes gravity dominates motion.
	 */
	const double transverseGravity = sqrt(
		(double)accelAxes.y * accelAxes.y
		+ (double)accelAxes.z * accelAxes.z);
	const double pitchDeg = atan2((double)accelAxes.x, transverseGravity)
		* RAD_TO_DEG;
	/* 【坡度控制】记录每次分类：+1 上坡，-1 下坡，0 平地。
	 * 只采集正常行驶中的读数，避免暂停、倒车、转向污染窗口。
	 */
	if (!sensors.buttons.B0 && !sensors.buttons.B1
		&& (state == APPROACH_RAMP || state == CLIMB_RAMP
			|| state == DRIVE_ACROSS_TOP || state == DESCEND_RAMP
			|| state == DRIVE_ON_LEVEL)){
		pitchSamples[pitchSampleIndex] = (pitchDeg >= PITCH_THRESHOLD_DEG) ? 1
			: ((pitchDeg <= -PITCH_THRESHOLD_DEG) ? -1 : 0);
		pitchSampleIndex = (pitchSampleIndex + 1) % PITCH_WINDOW_SAMPLE_COUNT;
		if (pitchSampleCount < PITCH_WINDOW_SAMPLE_COUNT){
			pitchSampleCount++;
		}
	}
	int uphillVotes = 0;
	int downhillVotes = 0;
	int levelVotes = 0;
	for (int sampleIndex = 0; sampleIndex < pitchSampleCount; sampleIndex++){
		if (pitchSamples[sampleIndex] == 1){ uphillVotes++; }
		else if (pitchSamples[sampleIndex] == -1){ downhillVotes++; }
		else{ levelVotes++; }
	}
	const bool pitchWindowReady = (pitchSampleCount == PITCH_WINDOW_SAMPLE_COUNT);
	const bool uphillSlope = pitchWindowReady && uphillVotes >= PITCH_REQUIRED_SAMPLE_COUNT;
	const bool downhillSlope = pitchWindowReady && downhillVotes >= PITCH_REQUIRED_SAMPLE_COUNT;
	const bool levelGround = pitchWindowReady && levelVotes >= PITCH_REQUIRED_SAMPLE_COUNT;
	/* 【坡度控制】计算 Y 的 5 次滑动平均，供方向纠偏使用。
	 * Rolling mean of the latest five raw Y readings, updated every call.
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
	/* 【Cliff 检测】左、中、右任一悬崖传感器触发即视为悬崖。 */
	const bool cliffDetected = sensors.cliffLeft
		|| sensors.cliffCenter
		|| sensors.cliffRight;


	/* 【通用按钮控制】B1 重置；B0 启动、暂停、恢复，优先于避让与坡度控制。
	 * B1 is a reset button: stop and require a fresh B0 press to start. */
	if (sensors.buttons.B1){
		ySampleSum = 0.0;
		ySampleIndex = 0;
		ySampleCount = 0;
		state = UNPAUSE_WAIT_BUTTON_PRESS;
		unpausedState = APPROACH_RAMP;
		pitchSampleCount = 0;
		pitchSampleIndex = 0;
	}
	else if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0				// pause button
		){
		pitchSampleCount = 0;
		pitchSampleIndex = 0;
		switch (state){
		case INITIAL:
			ySampleSum = 0.0;
			ySampleIndex = 0;
			ySampleCount = 0;
			/* Start every run from the beginning of the straight course. */
			unpausedState = APPROACH_RAMP;
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
	/* 【Cliff 状态切换】检测悬崖 -> 倒车达到距离阈值 -> 转向达到角度阈值 -> 恢复原阶段。
	 * 优先于坡度状态切换；倒车、转向期间不重复触发同一避让动作。
	 */
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
		pitchSampleCount = 0;
		pitchSampleIndex = 0;
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
	/* 【坡度控制】最近 8 次中 7 次下坡，可从正常行驶阶段直接进入下坡。 */
	else if (downhillSlope
		&& state != DESCEND_RAMP
		&& state != CLIFF_BACKUP
		&& state != CLIFF_TURN_RIGHT
		&& state != CLIFF_TURN_LEFT){
		state = DESCEND_RAMP;
		pitchSampleCount = 0;
		pitchSampleIndex = 0;
	}
	/* 【坡度控制】接近坡道 -> 上坡 -> 坡顶 -> 下坡 -> 平地，8 次中 7 次确认。 */
	else{
		bool transitionCondition = false;
		robotState_t nextState = state;

		switch (state){
		case APPROACH_RAMP:
			transitionCondition = uphillSlope;
			nextState = CLIMB_RAMP;
			break;
		case CLIMB_RAMP:
			transitionCondition = levelGround;
			nextState = DRIVE_ACROSS_TOP;
			break;
		case DRIVE_ACROSS_TOP:
			transitionCondition = downhillSlope;
			nextState = DESCEND_RAMP;
			break;
		case DESCEND_RAMP:
			transitionCondition = levelGround;
			nextState = DRIVE_ON_LEVEL;
			break;
		default:
			transitionCondition = false;
			break;
		}

		if (transitionCondition){
			state = nextState;
			pitchSampleCount = 0;
			pitchSampleIndex = 0;
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

	/* 【坡度控制动作】按当前行驶阶段设置两轮基础速度。 */
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

	/* 【Cliff 动作】弧线倒车与原地转向的轮速设置。 */
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

	/* 【坡度控制动作：Y 差速纠偏】在基础轮速之后执行，覆盖两轮速度。
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


	/* 【通用输出】最后应用固定轮速补偿和限速，作用于所有动作。
	 * Apply calibration to final commands, including reverse and turns.
	 * A stopped wheel stays stopped; saturation can reduce the boost.
	 */
	*pLeftWheelSpeed = trimWheelSpeed(leftWheelSpeed,
		(WHEEL_SPEED_TRIM < 0.0) ? -WHEEL_SPEED_TRIM : 0.0, maxWheelSpeed);
	*pRightWheelSpeed = trimWheelSpeed(rightWheelSpeed,
		(WHEEL_SPEED_TRIM > 0.0) ? WHEEL_SPEED_TRIM : 0.0, maxWheelSpeed);

	(void)isSimulator;
}
