/*
 * WS5 + 地面避障完整控制器（加速度单位 g，距离 mm，轮速 mm/s）。
 * 流程：接近坡道 -> 上坡 -> 坡顶/直接下坡 -> 下坡 -> 平地定距 -> 停车锁止。
 * 坡上不响应 bumper，仅做悬崖检测和坡度纠偏；地面才启用碰撞避障。
 * 本文件替换工程中的原状态机源文件，不与其他同名接口实现一起编译。
 */
#include "KobukiNavigationStateChart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 【坡度判断】原始读数投票，不平均、不要求连续满足。 */
#define RAD_TO_DEG                       57.29577951308232
#define PITCH_THRESHOLD_DEG              5.0
#define LEVEL_TILT_DEG                   3.0  /* 整体倾角：同时约束前后和横向 */
#define PITCH_WINDOW_SAMPLE_COUNT        8
#define PITCH_REQUIRED_SAMPLE_COUNT      7
#define Y_ALIGNMENT_THRESHOLD_G         0.01
#define Y_FILTER_SAMPLE_COUNT            5
#define Y_REQUIRED_SAMPLE_COUNT          3

/* 【路线轮速】先按阶段选择，再应用 Y 纠偏和固定轮速补偿。 */
#define APPROACH_SPEED_MM_S              200
#define CLIMB_SPEED_MM_S                 200
#define TOP_SPEED_MM_S                   150
#define DESCEND_SPEED_MM_S               100
#define FINISH_SPEED_MM_S                200
#ifndef FINISH_DISTANCE_MM
#define FINISH_DISTANCE_MM               200  /* 下坡确认平地后，正常直行的距离 */
#endif
#define UPHILL_ALIGN_OUTER_SPEED_MM_S    200
#define UPHILL_ALIGN_INNER_SPEED_MM_S    150
#define DOWNHILL_ALIGN_OUTER_SPEED_MM_S  100
#define DOWNHILL_ALIGN_INNER_SPEED_MM_S  75

/* 【地面碰撞避障】左撞右转、右撞左转、中央/双侧碰撞交替转向。 */
#define OBSTACLE_BACKUP_DISTANCE_MM      120
#define OBSTACLE_CLEAR_DISTANCE_MM       300
#define OBSTACLE_TURN_ANGLE_DEG          80
#define OBSTACLE_REVERSE_SPEED_MM_S      120
#define OBSTACLE_TURN_SPEED_MM_S         110
#define HEADING_TOLERANCE_DEG            5
#define REORIENT_MIN_SPEED_MM_S          20
#define REORIENT_GAIN_MM_S_PER_DEG       3

/* 【悬崖避让】保留 WS5 的弧线倒车 60 mm、原地转向 30 度。 */
#define CLIFF_BACKUP_DISTANCE_MM         60
#define CLIFF_TURN_ANGLE_DEG              30
#define CLIFF_BACKUP_FAST_MM_S           80
#define CLIFF_BACKUP_SLOW_MM_S           50
#define CLIFF_TURN_SPEED_MM_S            80
#ifndef WHEEL_SPEED_TRIM
#define WHEEL_SPEED_TRIM                 0.00 /* 负值补左轮，正值补右轮 */
#endif

#if PITCH_REQUIRED_SAMPLE_COUNT <= 0 || PITCH_REQUIRED_SAMPLE_COUNT > PITCH_WINDOW_SAMPLE_COUNT
#error Invalid_pitch_vote_parameters
#endif
#if Y_REQUIRED_SAMPLE_COUNT <= 0 || Y_REQUIRED_SAMPLE_COUNT > Y_FILTER_SAMPLE_COUNT
#error Invalid_Y_vote_parameters
#endif
#if FINISH_DISTANCE_MM <= 0
#error Finish_distance_must_be_positive
#endif

typedef enum {
    APPROACH_RAMP = 0, CLIMB_RAMP, DRIVE_ACROSS_TOP,
    DESCEND_RAMP, FINISH_STRAIGHT, FINISHED
} routePhase_t;

typedef enum {
    ROUTE_DRIVE = 0, OBSTACLE_BACKUP, OBSTACLE_TURN,
    OBSTACLE_CLEAR, OBSTACLE_REORIENT, CLIFF_BACKUP, CLIFF_TURN
} motionState_t;

typedef enum {
    WAIT_START_PRESS = 0, WAIT_START_RELEASE, RUNNING, WAIT_PAUSE_RELEASE
} buttonState_t;

typedef enum { SAMPLE_UNKNOWN = 0, SAMPLE_UPHILL, SAMPLE_DOWNHILL, SAMPLE_LEVEL } slopeSample_t;

typedef struct {
    bool initialized;
    bool hasStarted;
    routePhase_t phase;
    motionState_t motion;
    buttonState_t button;
    bool turnRight;
    bool centreTurnRight;
    int32_t routeHeading;
    int32_t maneuverDistance;
    int32_t maneuverAngle;
    int32_t lastDistance;
    int32_t finishDistance;
    bool countFinishDistance;
    slopeSample_t pitchSamples[PITCH_WINDOW_SAMPLE_COUNT];
    int pitchIndex;
    int pitchCount;
    int ySamples[Y_FILTER_SAMPLE_COUNT];
    int yIndex;
    int yCount;
} navigation_t;

/* 【通用控制】所有轮速均受调用方 maxWheelSpeed 限制。 */
static int16_t limitSpeed(const int16_t speed, const int16_t maximum)
{
    if (maximum <= 0) { return 0; }
    return speed < maximum ? speed : maximum;
}

static int16_t trimWheelSpeed(const int16_t speed, const double boost,
    const int16_t maximum)
{
    double magnitude = floor(fabs((double)speed) * (1.0 + boost) + 0.5);
    if (maximum <= 0) { return 0; }
    if (magnitude > maximum) { magnitude = maximum; }
    return (int16_t)(speed < 0 ? -magnitude : magnitude);
}

/* 正误差表示向左偏，应右转；归一化后可处理跨 0/360 度和累计多圈。 */
static int32_t signedAngleError(const int32_t current, const int32_t target)
{
    int32_t error = (current % 360 - target % 360) % 360;
    if (error > 180) { error -= 360; }
    else if (error < -180) { error += 360; }
    return error;
}

/* 暂停、避让、路线切换后重新收满窗口，不沿用旧的坡度或 Y 证据。 */
static void resetVotes(navigation_t * const nav)
{
    nav->pitchIndex = nav->pitchCount = 0;
    nav->yIndex = nav->yCount = 0;
}

static void resetRun(navigation_t * const nav, const int32_t distance,
    const int32_t angle)
{
    memset(nav, 0, sizeof(*nav));
    nav->initialized = true;
    nav->centreTurnRight = true;
    nav->routeHeading = angle;
    nav->lastDistance = distance;
}

static bool onGround(const routePhase_t phase)
{
    return phase == APPROACH_RAMP || phase == FINISH_STRAIGHT;
}

/* 【坡度判断】横着处于斜坡不能算平地；过小/过大的合加速度不投有效票。 */
static slopeSample_t classifySlope(const accelerometer_t accel)
{
    double lateral = sqrt(accel.y * accel.y + accel.z * accel.z);
    double magnitude = sqrt(accel.x * accel.x + lateral * lateral);
    double pitch;
    double tilt;
    if (!(magnitude >= 0.5 && magnitude <= 1.5)) { return SAMPLE_UNKNOWN; }
    pitch = atan2(accel.x, lateral) * RAD_TO_DEG;
    tilt = atan2(sqrt(accel.x * accel.x + accel.y * accel.y),
        fabs(accel.z)) * RAD_TO_DEG;
    if (pitch >= PITCH_THRESHOLD_DEG) { return SAMPLE_UPHILL; }
    if (pitch <= -PITCH_THRESHOLD_DEG) { return SAMPLE_DOWNHILL; }
    if (tilt <= LEVEL_TILT_DEG) { return SAMPLE_LEVEL; }
    return SAMPLE_UNKNOWN;
}

/* 【路线阶段】只在正常前进或地面绕行前进时判断，倒车/转向不推进阶段。 */
static void updateRoute(navigation_t * const nav, const accelerometer_t accel,
    const int32_t angle)
{
    int i;
    int up = 0, down = 0, level = 0;
    routePhase_t next = nav->phase;
    nav->pitchSamples[nav->pitchIndex] = classifySlope(accel);
    nav->pitchIndex = (nav->pitchIndex + 1) % PITCH_WINDOW_SAMPLE_COUNT;
    if (nav->pitchCount < PITCH_WINDOW_SAMPLE_COUNT) { nav->pitchCount++; }
    if (nav->pitchCount != PITCH_WINDOW_SAMPLE_COUNT) { return; }
    for (i = 0; i < PITCH_WINDOW_SAMPLE_COUNT; i++) {
        if (nav->pitchSamples[i] == SAMPLE_UPHILL) { up++; }
        else if (nav->pitchSamples[i] == SAMPLE_DOWNHILL) { down++; }
        else if (nav->pitchSamples[i] == SAMPLE_LEVEL) { level++; }
    }
    switch (nav->phase) {
    case APPROACH_RAMP:
        if (up >= PITCH_REQUIRED_SAMPLE_COUNT) { next = CLIMB_RAMP; }
        break;
    case CLIMB_RAMP:
        if (down >= PITCH_REQUIRED_SAMPLE_COUNT) { next = DESCEND_RAMP; }
        else if (level >= PITCH_REQUIRED_SAMPLE_COUNT) { next = DRIVE_ACROSS_TOP; }
        break;
    case DRIVE_ACROSS_TOP:
        if (down >= PITCH_REQUIRED_SAMPLE_COUNT) { next = DESCEND_RAMP; }
        break;
    case DESCEND_RAMP:
        if (level >= PITCH_REQUIRED_SAMPLE_COUNT) { next = FINISH_STRAIGHT; }
        break;
    default:
        break;
    }
    if (next != nav->phase) {
        nav->phase = next;
        resetVotes(nav);
        /* 上坡期间关闭原地面避障动作；悬崖动作在本函数调用前已排除。 */
        if (!onGround(next)) { nav->motion = ROUTE_DRIVE; }
        if (next == FINISH_STRAIGHT) {
            nav->finishDistance = 0;
            nav->countFinishDistance = false;
            nav->routeHeading = angle;
        }
    }
}

/* 【Y 差速纠偏】五次中三次同方向；只覆盖正常上、下坡的直行轮速。 */
static int updateY(navigation_t * const nav, const double y)
{
    int i;
    int positive = 0, negative = 0;
    nav->ySamples[nav->yIndex] = y >= Y_ALIGNMENT_THRESHOLD_G ? 1
        : (y <= -Y_ALIGNMENT_THRESHOLD_G ? -1 : 0);
    nav->yIndex = (nav->yIndex + 1) % Y_FILTER_SAMPLE_COUNT;
    if (nav->yCount < Y_FILTER_SAMPLE_COUNT) { nav->yCount++; }
    if (nav->yCount != Y_FILTER_SAMPLE_COUNT) { return 0; }
    for (i = 0; i < Y_FILTER_SAMPLE_COUNT; i++) {
        if (nav->ySamples[i] > 0) { positive++; }
        else if (nav->ySamples[i] < 0) { negative++; }
    }
    if (positive >= Y_REQUIRED_SAMPLE_COUNT) { return 1; }
    if (negative >= Y_REQUIRED_SAMPLE_COUNT) { return -1; }
    return 0;
}

/* 【地面避障】重复碰撞保留本轮原路线航向；新一轮从当前有效航向开始。 */
static void beginObstacle(navigation_t * const nav, const KobukiSensors_t sensors,
    const int32_t distance, const int32_t angle)
{
    bool left = sensors.bumps_wheelDrops.bumpLeft;
    bool right = sensors.bumps_wheelDrops.bumpRight;
    if (nav->motion == ROUTE_DRIVE) { nav->routeHeading = angle; }
    if (left && !right) { nav->turnRight = true; }
    else if (right && !left) { nav->turnRight = false; }
    else {
        nav->turnRight = nav->centreTurnRight;
        nav->centreTurnRight = !nav->centreTurnRight;
    }
    nav->maneuverDistance = distance;
    nav->motion = OBSTACLE_BACKUP;
    resetVotes(nav);
}

/* 【悬崖避让】优先于地面避障；取消被打断的避障动作，但保留路线阶段。 */
static void beginCliff(navigation_t * const nav, const KobukiSensors_t sensors,
    const int32_t distance)
{
    nav->turnRight = sensors.cliffLeft || sensors.cliffCenter;
    nav->maneuverDistance = distance;
    nav->motion = CLIFF_BACKUP;
    resetVotes(nav);
}

/* 【统一输出与终点计距】只把正常直行命令覆盖的区间标为可累计。 */
static void outputSpeeds(navigation_t * const nav, const int16_t left,
    const int16_t right, const int16_t maximum, const int32_t distance,
    int16_t * const pRight, int16_t * const pLeft)
{
    *pLeft = trimWheelSpeed(left, WHEEL_SPEED_TRIM < 0 ? -WHEEL_SPEED_TRIM : 0, maximum);
    *pRight = trimWheelSpeed(right, WHEEL_SPEED_TRIM > 0 ? WHEEL_SPEED_TRIM : 0, maximum);
    nav->lastDistance = distance;
    nav->countFinishDistance = nav->phase == FINISH_STRAIGHT
        && nav->button == RUNNING && nav->motion == ROUTE_DRIVE
        && *pLeft > 0 && *pRight > 0;
}

void KobukiNavigationStatechart(const int16_t maxWheelSpeed,
    const int32_t netDistance, const int32_t netAngle,
    const KobukiSensors_t sensors, const accelerometer_t accelAxes,
    int16_t * const pRightWheelSpeed, int16_t * const pLeftWheelSpeed,
    const bool isSimulator)
{
    static navigation_t nav;
    int16_t left = 0, right = 0, speed;
    int correction;
    int32_t error;
    int32_t reorientSpeed;
    int64_t delta;
    bool cliff = sensors.cliffLeft || sensors.cliffCenter || sensors.cliffRight;
    bool bumper = sensors.bumps_wheelDrops.bumpLeft
        || sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpRight;
    bool turnCompleted;
    (void)isSimulator;

    /* B1 重置后仍为停车等待状态，必须重新按下并松开 B0。 */
    if (!nav.initialized || sensors.buttons.B1) {
        resetRun(&nav, netDistance, netAngle);
        /* 重置时若 B0 已按住，先等它释放，避免把旧按键当成新启动。 */
        if (sensors.buttons.B0) { nav.button = WAIT_PAUSE_RELEASE; }
        outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
            pRightWheelSpeed, pLeftWheelSpeed);
        return;
    }

    /* 用上一周期的命令决定计距：只加前进距离，不把倒车取绝对值。 */
    if (nav.countFinishDistance) {
        delta = (int64_t)netDistance - nav.lastDistance;
        if (delta > 0) {
            if (delta >= FINISH_DISTANCE_MM - nav.finishDistance) {
                nav.finishDistance = FINISH_DISTANCE_MM;
            }
            else { nav.finishDistance += (int32_t)delta; }
        }
    }
    if (nav.phase == FINISH_STRAIGHT && nav.finishDistance >= FINISH_DISTANCE_MM) {
        nav.phase = FINISHED;
    }
    /* 【终点锁止】优先于 B0、bumper 和悬崖；任何输入都不能自动复跑。 */
    if (nav.phase == FINISHED) {
        outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
            pRightWheelSpeed, pLeftWheelSpeed);
        return;
    }

    /* 【B0 按钮】按下立即暂停；释放、再次按下并释放后才恢复。 */
    switch (nav.button) {
    case RUNNING:
        if (sensors.buttons.B0) { nav.button = WAIT_PAUSE_RELEASE; resetVotes(&nav); }
        break;
    case WAIT_PAUSE_RELEASE:
        if (!sensors.buttons.B0) { nav.button = WAIT_START_PRESS; }
        break;
    case WAIT_START_PRESS:
        if (sensors.buttons.B0) { nav.button = WAIT_START_RELEASE; }
        break;
    case WAIT_START_RELEASE:
        if (!sensors.buttons.B0) {
            nav.button = RUNNING;
            resetVotes(&nav);
            if (!nav.hasStarted) { nav.routeHeading = netAngle; nav.hasStarted = true; }
        }
        break;
    }
    if (nav.button != RUNNING) {
        outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
            pRightWheelSpeed, pLeftWheelSpeed);
        return;
    }

    /* 【悬崖优先】检测当次就倒车；不采集避让中的坡度读数。 */
    if (cliff && nav.motion != CLIFF_BACKUP && nav.motion != CLIFF_TURN) {
        beginCliff(&nav, sensors, netDistance);
    }
    if (!cliff && (nav.motion == ROUTE_DRIVE || nav.motion == OBSTACLE_CLEAR)
        && nav.phase != FINISH_STRAIGHT) {
        updateRoute(&nav, accelAxes, netAngle);
    }
    if (nav.motion == CLIFF_BACKUP
        && (int64_t)nav.maneuverDistance - netDistance >= CLIFF_BACKUP_DISTANCE_MM) {
        nav.maneuverAngle = netAngle;
        nav.motion = CLIFF_TURN;
    }
    else if (nav.motion == CLIFF_TURN
        && abs(signedAngleError(netAngle, nav.maneuverAngle)) >= CLIFF_TURN_ANGLE_DEG) {
        if (cliff) { beginCliff(&nav, sensors, netDistance); }
        else {
            nav.motion = ROUTE_DRIVE;
            nav.routeHeading = netAngle; /* 不回到刚才朝向悬崖的旧航向 */
            resetVotes(&nav);
        }
    }

    /* 【地面避障】上坡、坡顶、下坡完全跳过 bumper 处理。 */
    if (onGround(nav.phase) && nav.motion != CLIFF_BACKUP && nav.motion != CLIFF_TURN) {
        turnCompleted = nav.motion == OBSTACLE_TURN
            && abs(signedAngleError(netAngle, nav.maneuverAngle)) >= OBSTACLE_TURN_ANGLE_DEG;
        if (bumper && ((nav.motion != OBSTACLE_BACKUP && nav.motion != OBSTACLE_TURN)
                || turnCompleted)) {
            beginObstacle(&nav, sensors, netDistance, netAngle);
        }
        else if (nav.motion == OBSTACLE_BACKUP
            && (int64_t)nav.maneuverDistance - netDistance >= OBSTACLE_BACKUP_DISTANCE_MM) {
            nav.maneuverAngle = netAngle;
            nav.motion = OBSTACLE_TURN;
        }
        else if (turnCompleted) {
            nav.maneuverDistance = netDistance;
            nav.motion = OBSTACLE_CLEAR;
        }
        else if (nav.motion == OBSTACLE_CLEAR
            && (int64_t)netDistance - nav.maneuverDistance >= OBSTACLE_CLEAR_DISTANCE_MM) {
            nav.motion = OBSTACLE_REORIENT;
        }
        else if (nav.motion == OBSTACLE_REORIENT
            && abs(signedAngleError(netAngle, nav.routeHeading)) <= HEADING_TOLERANCE_DEG) {
            nav.motion = ROUTE_DRIVE;
            resetVotes(&nav);
        }
    }

    /* 【动作输出】避让轮速不会被坡度纠偏覆盖。 */
    switch (nav.motion) {
    case ROUTE_DRIVE:
        speed = APPROACH_SPEED_MM_S;
        if (nav.phase == CLIMB_RAMP) { speed = CLIMB_SPEED_MM_S; }
        else if (nav.phase == DRIVE_ACROSS_TOP) { speed = TOP_SPEED_MM_S; }
        else if (nav.phase == DESCEND_RAMP) { speed = DESCEND_SPEED_MM_S; }
        else if (nav.phase == FINISH_STRAIGHT) { speed = FINISH_SPEED_MM_S; }
        left = right = limitSpeed(speed, maxWheelSpeed);
        if (nav.phase == CLIMB_RAMP || nav.phase == DESCEND_RAMP) {
            correction = updateY(&nav, accelAxes.y);
            if (correction != 0) {
                if (nav.phase == CLIMB_RAMP) {
                    left = limitSpeed(correction > 0 ? UPHILL_ALIGN_OUTER_SPEED_MM_S
                        : UPHILL_ALIGN_INNER_SPEED_MM_S, maxWheelSpeed);
                    right = limitSpeed(correction > 0 ? UPHILL_ALIGN_INNER_SPEED_MM_S
                        : UPHILL_ALIGN_OUTER_SPEED_MM_S, maxWheelSpeed);
                }
                else {
                    left = limitSpeed(correction > 0 ? DOWNHILL_ALIGN_INNER_SPEED_MM_S
                        : DOWNHILL_ALIGN_OUTER_SPEED_MM_S, maxWheelSpeed);
                    right = limitSpeed(correction > 0 ? DOWNHILL_ALIGN_OUTER_SPEED_MM_S
                        : DOWNHILL_ALIGN_INNER_SPEED_MM_S, maxWheelSpeed);
                }
            }
        }
        break;
    case OBSTACLE_BACKUP:
        left = right = -limitSpeed(OBSTACLE_REVERSE_SPEED_MM_S, maxWheelSpeed);
        break;
    case OBSTACLE_TURN:
        speed = limitSpeed(OBSTACLE_TURN_SPEED_MM_S, maxWheelSpeed);
        left = nav.turnRight ? speed : -speed;
        right = -left;
        break;
    case OBSTACLE_CLEAR:
        left = right = limitSpeed(APPROACH_SPEED_MM_S, maxWheelSpeed);
        break;
    case OBSTACLE_REORIENT:
        error = signedAngleError(netAngle, nav.routeHeading);
        if (abs(error) <= HEADING_TOLERANCE_DEG) { break; }
        reorientSpeed = abs(error) * REORIENT_GAIN_MM_S_PER_DEG;
        if (reorientSpeed < REORIENT_MIN_SPEED_MM_S) { reorientSpeed = REORIENT_MIN_SPEED_MM_S; }
        speed = limitSpeed(OBSTACLE_TURN_SPEED_MM_S, maxWheelSpeed);
        if (reorientSpeed > speed) { reorientSpeed = speed; }
        left = (int16_t)(error > 0 ? reorientSpeed : -reorientSpeed);
        right = -left;
        break;
    case CLIFF_BACKUP:
        left = -limitSpeed(nav.turnRight ? CLIFF_BACKUP_SLOW_MM_S : CLIFF_BACKUP_FAST_MM_S,
            maxWheelSpeed);
        right = -limitSpeed(nav.turnRight ? CLIFF_BACKUP_FAST_MM_S : CLIFF_BACKUP_SLOW_MM_S,
            maxWheelSpeed);
        break;
    case CLIFF_TURN:
        speed = limitSpeed(CLIFF_TURN_SPEED_MM_S, maxWheelSpeed);
        left = nav.turnRight ? speed : -speed;
        right = -left;
        break;
    }
    outputSpeeds(&nav, left, right, maxWheelSpeed, netDistance,
        pRightWheelSpeed, pLeftWheelSpeed);
}
