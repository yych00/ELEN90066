/*
 * WS5 + 地面避障完整控制器：版本 1.1。
 * 基线：提交 3212560 的 KobukiNavigationStateChart_combined_debug.c。
 * 停车只接受：进入下坡置 downhillFlag -> 下坡后确认平地直接停车锁止。
 * 初始平面、上坡后的坡顶平面均不置停车标志；B1 重置两个标志。
 * 坡度采用 8 中 6 投票，保留 Y 纠偏与轮速；combined.c 保持原文件。
 * 正常直行、绕障直行都检测上坡；确认上坡后立即取消地面避障。
 * 碰撞同时读到上坡时先停车，重新收满 8 次读数：6 票上坡才进入爬坡。
 * 确认失败且碰撞仍在才倒车；无效读数也占窗口，避免无限等待。
 * 接近坡道的碰撞避障采用 new5_downhill_stop_jzcv2.c 的动作与航向控制。
 * 基础行驶及避障轮速保持原来的约 70%；纠偏使用原参数，输出端不再统一缩放。
 * 加速度单位 g，距离 mm，轮速 mm/s。
 * 流程：接近坡道 -> 上坡 -> 坡顶/直接下坡 -> 下坡 -> 平地直接停车锁止。
 * 坡上不响应 bumper，仅做悬崖检测和坡度纠偏；地面才启用碰撞避障。
 * 坡道悬崖避让保留坡度投票，动作期间暂停采样，结束后接着判断下坡/落地。
 * 本文件替换工程中的原状态机源文件，不与其他同名接口实现一起编译。
 */
#include "KobukiNavigationStateChart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 【坡度判断】原始读数投票，不平均、不要求连续满足。 */
#define RAD_TO_DEG                       57.29577951308232
#define PITCH_THRESHOLD_DEG              5.0
#define LEVEL_TILT_DEG                   4.5  /* 整体倾角：同时约束前后和横向 */
#define PITCH_WINDOW_SAMPLE_COUNT        8
#define PITCH_REQUIRED_SAMPLE_COUNT      6
#define Y_ALIGNMENT_THRESHOLD_G          0.01
#define Y_FILTER_SAMPLE_COUNT            5
#define Y_REQUIRED_SAMPLE_COUNT          3
#define Y_CORRECTION_MAX_DURATION_MS     2000  /* 持续纠偏最大时间：2秒超时强制退出 */
#define Y_CORRECTION_COOLDOWN_MS         1000  /* 超时退出后的平稳直行冷却时间 */

/* 【路线轮速】基础值已降至原来的 70%，先选阶段，再应用纠偏和固定补偿。 */
#define APPROACH_SPEED_MM_S              280
#define CLIMB_SPEED_MM_S                 280
#define TOP_SPEED_MM_S                   210
#define DESCEND_SPEED_MM_S               140
#define FINISH_SPEED_MM_S                280
#ifndef FINISH_DISTANCE_MM
#define FINISH_DISTANCE_MM               200  /* 下坡确认平地后，正常直行的距离 */
#endif
/* Y 差速纠偏使用降速前的轮速，不额外乘 70%。 */
#define UPHILL_ALIGN_OUTER_SPEED_MM_S    400
#define UPHILL_ALIGN_INNER_SPEED_MM_S    300
#define DOWNHILL_ALIGN_OUTER_SPEED_MM_S  200
#define DOWNHILL_ALIGN_INNER_SPEED_MM_S  150

/* 【地面碰撞避障】左撞右转、右撞左转、中央/双侧碰撞交替转向。 */
#define OBSTACLE_BACKUP_DISTANCE_MM      120
#define OBSTACLE_CLEAR_DISTANCE_MM       300
#define OBSTACLE_TURN_ANGLE_DEG          80
#define OBSTACLE_REVERSE_SPEED_MM_S      168
#define OBSTACLE_TURN_SPEED_MM_S         154
#define HEADING_TOLERANCE_DEG            5
#define REORIENT_MIN_SPEED_MM_S          40
#define REORIENT_GAIN_MM_S_PER_DEG       6

/* 【前半段航向保持】使用降速前的纠偏增益和限幅，在较低基础轮速上计算差速。 */
#ifndef HEADING_ANGLE_SIGN
#define HEADING_ANGLE_SIGN               1
#endif
#define HEADING_KP_MM_S_PER_DEG           2.5
#define HEADING_KD_MM_S_PER_DEG_STEP      8.0
#define HEADING_KI_MM_S_PER_DEG_SAMPLE    0.015
#define HEADING_MAX_INTEGRAL_MM_S        12.0
#define HEADING_YAW_RATE_FILTER_ALPHA    0.25
#define HEADING_CORRECTION_STEP_MM_S     3.0
#define HEADING_DEADBAND_DEG             1
#define HEADING_MAX_CORRECTION_MM_S      40

/* 【悬崖避让】保留 WS5 的弧线倒车 60 mm、原地转向 30 度。 */
#define CLIFF_BACKUP_DISTANCE_MM         60
#define CLIFF_TURN_ANGLE_DEG              30
#define CLIFF_BACKUP_FAST_MM_S           112
#define CLIFF_BACKUP_SLOW_MM_S           70
#define CLIFF_TURN_SPEED_MM_S            112
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
    int32_t clearHeading;
    bool headingControlActive;
    motionState_t headingControlMotion;
    int32_t previousControlHeading;
    int32_t previousControlAngle;
    double filteredYawStep;
    double previousCorrection;
    double headingIntegral;
    int32_t maneuverDistance;
    int32_t maneuverAngle;
    int32_t lastDistance;
    int32_t finishDistance;
    bool countFinishDistance;
    bool downhillFlag; /* 只在路线进入 DESCEND_RAMP 时置位，坡顶平地不能置位。 */
    bool landingFlag;  /* 只在已进入下坡之后再次确认平地时置位，B1 清除。 */
    bool confirmingUphill; /* 坡道入口碰撞时停车确认；不改变路线或停车标志。 */
    slopeSample_t pitchSamples[PITCH_WINDOW_SAMPLE_COUNT];
    int pitchIndex;
    int pitchCount;
    int ySamples[Y_FILTER_SAMPLE_COUNT];
    int yIndex;
    int yCount;
    int32_t yCorrectionDurationMs;
    int32_t yCooldownMs;
    int32_t lastTimeStamp; /* 用有符号 32 位保存完整的 16 位时间戳。 */
    int levelGroundCount;  /* 拿到下坡flag后，平地持续检测计数器 */
} navigation_t;

/* 【通用控制】已去掉调用方 maxWheelSpeed 速度上限限制。 */
static int16_t limitSpeed(const int16_t speed, const int16_t maximum)
{
    (void)maximum;
    return speed;
}

static int16_t trimWheelSpeed(const int16_t speed, const double boost,
    const int16_t maximum)
{
    double magnitude = floor(fabs((double)speed) * (1.0 + boost) + 0.5);
    (void)maximum;
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

/* 【倒车完成判据】兼容倒车时里程递减的净里程，以及始终递增的累计里程。
 * 距离达到阈值立即切换转向，不依赖计数方向；先升为 int64 再取绝对值，
 * 避免两个 int32 读数相减或 abs(INT32_MIN) 溢出。里程仍需由调用方更新。
 */
static bool backupDistanceReached(const int32_t current, const int32_t start,
    const int32_t required)
{
    int64_t travelled = (int64_t)current - start;
    if (travelled < 0) { travelled = -travelled; }
    return travelled >= required;
}

/* 暂停、地面碰撞、路线切换后重新收满坡度和 Y 窗口。 */
static void resetVotes(navigation_t * const nav)
{
    nav->confirmingUphill = false;
    nav->pitchIndex = nav->pitchCount = 0;
    nav->yIndex = nav->yCount = 0;
    nav->yCorrectionDurationMs = 0;
    nav->yCooldownMs = 0;
    nav->levelGroundCount = 0;
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

/* 【悬崖窗口】坡道上保留坡度票和写入位置，避免下坡/落地确认反复重来。
 * 入口确认、Y 投票与纠偏计时均清除；地面仍丢弃未确认的坡度证据。
 * 悬崖倒车和转向期间不调用 updateRoute，不会写入新的坡度票。
 */
static void resetCliffVotes(navigation_t * const nav)
{
    nav->confirmingUphill = false;
    nav->yIndex = nav->yCount = 0;
    nav->yCorrectionDurationMs = 0;
    nav->yCooldownMs = 0;
    if (onGround(nav->phase)) {
        nav->pitchIndex = nav->pitchCount = 0;
    }
}

/* 【坡度判断】通常按整体倾角判平地；已确认下坡后按前后俯仰持续判落地。
 * 放宽落地判断时的合加速度范围与俯仰阈值，彻底消除死区并吸收接缝冲击。
 */
static slopeSample_t classifySlope(const accelerometer_t accel, const bool landingCheck)
{
    double lateral = sqrt(accel.y * accel.y + accel.z * accel.z);
    double magnitude = sqrt(accel.x * accel.x + lateral * lateral);
    double pitch;
    double tilt;
    double magMin = landingCheck ? 0.3 : 0.5;
    double magMax = landingCheck ? 2.2 : 1.5;
    if (!(magnitude >= magMin && magnitude <= magMax)) { return SAMPLE_UNKNOWN; }
    pitch = atan2(accel.x, lateral) * RAD_TO_DEG;
    tilt = atan2(sqrt(accel.x * accel.x + accel.y * accel.y),
        fabs(accel.z)) * RAD_TO_DEG;
    if (pitch >= PITCH_THRESHOLD_DEG) { return SAMPLE_UPHILL; }
    if (pitch <= -PITCH_THRESHOLD_DEG) { return SAMPLE_DOWNHILL; }
    if (landingCheck) {
        if (pitch > -PITCH_THRESHOLD_DEG && pitch < PITCH_THRESHOLD_DEG) {
            return SAMPLE_LEVEL;
        }
    } else {
        if (tilt <= LEVEL_TILT_DEG) { return SAMPLE_LEVEL; }
    }
    return SAMPLE_UNKNOWN;
}

/* 【路线阶段】正常/绕障直行及入口停车确认时采样，倒车/转向不推进阶段。 */
static void updateRoute(navigation_t * const nav, const accelerometer_t accel,
    const int32_t angle)
{
    int i;
    int up = 0, down = 0, level = 0;
    routePhase_t next = nav->phase;
    slopeSample_t currentSample = classifySlope(accel, nav->downhillFlag);

    nav->pitchSamples[nav->pitchIndex] = currentSample;
    nav->pitchIndex = (nav->pitchIndex + 1) % PITCH_WINDOW_SAMPLE_COUNT;
    if (nav->pitchCount < PITCH_WINDOW_SAMPLE_COUNT) { nav->pitchCount++; }

    for (i = 0; i < nav->pitchCount; i++) {
        if (nav->pitchSamples[i] == SAMPLE_UPHILL) { up++; }
        else if (nav->pitchSamples[i] == SAMPLE_DOWNHILL) { down++; }
        else if (nav->pitchSamples[i] == SAMPLE_LEVEL) { level++; }
    }

    /* 【获得下坡 flag】爬坡中或坡顶，检测到下坡趋势（4票下坡），立即锁存下坡标志 */
    if (!nav->downhillFlag && (nav->phase == CLIMB_RAMP || nav->phase == DRIVE_ACROSS_TOP)) {
        if (down >= 4) {
            nav->downhillFlag = true;
            next = DESCEND_RAMP;
        }
    }

    /* 【拿到下坡 flag 后的平地持续检测】
     * 只要已获得下坡 flag，在平地上全程持续检测；
     * 当车身脱离下坡（最新样本为平地）时持续累加；当下坡时清零。
     * 连续 3 拍或窗口满 4 票平地，直接判定到达平地停车锁止。
     */
    if (nav->downhillFlag) {
        if (currentSample == SAMPLE_LEVEL) {
            nav->levelGroundCount++;
        }
        else if (currentSample == SAMPLE_DOWNHILL) {
            nav->levelGroundCount = 0;
        }
        if (nav->levelGroundCount >= 3 || level >= 4) {
            next = FINISHED;
        }
    }

    switch (nav->phase) {
    case APPROACH_RAMP:
        if (up >= PITCH_REQUIRED_SAMPLE_COUNT) { next = CLIMB_RAMP; }
        break;
    case CLIMB_RAMP:
        if (down >= 4) { next = DESCEND_RAMP; }
        else if (level >= PITCH_REQUIRED_SAMPLE_COUNT) { next = DRIVE_ACROSS_TOP; }
        break;
    case DRIVE_ACROSS_TOP:
        if (down >= 4) { next = DESCEND_RAMP; }
        break;
    case DESCEND_RAMP:
        if (nav->downhillFlag && (nav->levelGroundCount >= 3 || level >= 4)) {
            next = FINISHED;
        }
        break;
    default:
        break;
    }

    if (next != nav->phase) {
        nav->phase = next;
        if (next == DESCEND_RAMP) { nav->downhillFlag = true; }
        resetVotes(nav);
        /* 上坡期间关闭原地面避障动作；悬崖动作在本函数调用前已排除。 */
        if (!onGround(next)) { nav->motion = ROUTE_DRIVE; }
        if (next == FINISH_STRAIGHT || next == FINISHED) {
            nav->landingFlag = true;
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

/* 【地面避障】前半段保留启动/恢复时的路线航向，按参考文件回到原路线。
 * 最后平面沿用本轮碰撞前的航向，避免影响已确认的终点直行方向。
 */
static void beginObstacle(navigation_t * const nav, const KobukiSensors_t sensors,
    const int32_t distance, const int32_t angle)
{
    bool left = sensors.bumps_wheelDrops.bumpLeft;
    bool right = sensors.bumps_wheelDrops.bumpRight;
    if (nav->phase == FINISH_STRAIGHT && nav->motion == ROUTE_DRIVE) {
        nav->routeHeading = angle;
    }
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
    resetCliffVotes(nav);
}

/* 【前半段直行/绕行纠偏】正常直行锁定路线航向，绕行锁定转向完成时的航向。
 * 保留配置的基础速度；已移除调用方轮速上限截断，保持两轮差速纠偏。
 * 和参考文件一样，KD 与积分按调用次数计算，调用周期变化会影响实际纠偏效果。
 */
static void maintainApproachHeading(navigation_t * const nav, const int32_t angle,
    const int16_t maximum, int16_t * const left, int16_t * const right)
{
    int32_t target;
    double error, yawStep = 0.0, correction, correctionLimit, baseSpeed;
    double requestedLeft, requestedRight;
    (void)maximum;
    if (nav->phase != APPROACH_RAMP || (nav->motion != ROUTE_DRIVE
            && nav->motion != OBSTACLE_CLEAR)) {
        nav->headingControlActive = false;
        return;
    }
    target = nav->motion == OBSTACLE_CLEAR ? nav->clearHeading : nav->routeHeading;
    error = HEADING_ANGLE_SIGN * signedAngleError(angle, target);
    baseSpeed = *left;
    correctionLimit = HEADING_MAX_CORRECTION_MM_S;
    if (!nav->headingControlActive || nav->headingControlMotion != nav->motion
        || nav->previousControlHeading != target) {
        nav->filteredYawStep = nav->previousCorrection = nav->headingIntegral = 0.0;
    }
    else {
        yawStep = HEADING_ANGLE_SIGN * signedAngleError(angle, nav->previousControlAngle);
        nav->filteredYawStep = HEADING_YAW_RATE_FILTER_ALPHA * yawStep
            + (1.0 - HEADING_YAW_RATE_FILTER_ALPHA) * nav->filteredYawStep;
    }
    nav->headingControlActive = true;
    nav->headingControlMotion = nav->motion;
    nav->previousControlHeading = target;
    nav->previousControlAngle = angle;
    if (fabs(error) <= 15.0 && baseSpeed > 0.0) {
        nav->headingIntegral += HEADING_KI_MM_S_PER_DEG_SAMPLE * error;
        if (nav->headingIntegral > HEADING_MAX_INTEGRAL_MM_S) {
            nav->headingIntegral = HEADING_MAX_INTEGRAL_MM_S;
        }
        if (nav->headingIntegral < -HEADING_MAX_INTEGRAL_MM_S) {
            nav->headingIntegral = -HEADING_MAX_INTEGRAL_MM_S;
        }
    }
    if (error > HEADING_DEADBAND_DEG) { error -= HEADING_DEADBAND_DEG; }
    else if (error < -HEADING_DEADBAND_DEG) { error += HEADING_DEADBAND_DEG; }
    else { error = 0.0; }
    correction = HEADING_KP_MM_S_PER_DEG * error
        + HEADING_KD_MM_S_PER_DEG_STEP * nav->filteredYawStep + nav->headingIntegral;
    if (correctionLimit > baseSpeed / 2.0) { correctionLimit = baseSpeed / 2.0; }
    if (correction > correctionLimit) { correction = correctionLimit; }
    if (correction < -correctionLimit) { correction = -correctionLimit; }
    if (correction > nav->previousCorrection + HEADING_CORRECTION_STEP_MM_S) {
        correction = nav->previousCorrection + HEADING_CORRECTION_STEP_MM_S;
    }
    if (correction < nav->previousCorrection - HEADING_CORRECTION_STEP_MM_S) {
        correction = nav->previousCorrection - HEADING_CORRECTION_STEP_MM_S;
    }
    if (correction > correctionLimit) { correction = correctionLimit; }
    if (correction < -correctionLimit) { correction = -correctionLimit; }
    nav->previousCorrection = correction;
    requestedLeft = baseSpeed + correction;
    requestedRight = baseSpeed - correction;
    if (requestedLeft < 0.0) { requestedLeft = 0.0; }
    if (requestedRight < 0.0) { requestedRight = 0.0; }
    *left = (int16_t)floor(requestedLeft + 0.5);
    *right = (int16_t)floor(requestedRight + 0.5);
}

/* 【统一输出与终点计距】只把正常直行命令覆盖的区间标为可累计。 */
static void outputSpeeds(navigation_t * const nav, const int16_t left,
    const int16_t right, const int16_t maximum, const int32_t distance,
    int16_t * const pRight, int16_t * const pLeft)
{
    *pLeft = trimWheelSpeed(left, WHEEL_SPEED_TRIM < 0 ? -WHEEL_SPEED_TRIM : 0, maximum);
    *pRight = trimWheelSpeed(right, WHEEL_SPEED_TRIM > 0 ? WHEEL_SPEED_TRIM : 0, maximum);
    nav->lastDistance = distance;
    nav->countFinishDistance = nav->landingFlag && nav->phase == FINISH_STRAIGHT
        && nav->button == RUNNING && nav->motion == ROUTE_DRIVE
        && *pLeft > 0 && *pRight > 0;
    if (nav->button != RUNNING) { nav->headingControlActive = false; }
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
    int32_t dtMs = 64;
    int32_t timeStampDiff;
    bool cliff = sensors.cliffLeft || sensors.cliffCenter || sensors.cliffRight;
    bool bumper = sensors.bumps_wheelDrops.bumpLeft
        || sensors.bumps_wheelDrops.bumpCenter || sensors.bumps_wheelDrops.bumpRight;
    bool turnCompleted;
    (void)isSimulator;

    if (sensors.TimeStamp != 0 && nav.lastTimeStamp != 0) {
        timeStampDiff = (int32_t)sensors.TimeStamp - nav.lastTimeStamp;
        /* 传感器时间戳从 65535 回到 0 时，显式补回一圈，代替无符号回绕。 */
        if (timeStampDiff < 0) { timeStampDiff += 65536L; }
        if (timeStampDiff > 0 && timeStampDiff < 500) { dtMs = timeStampDiff; }
    }
    nav.lastTimeStamp = sensors.TimeStamp;

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
    if (nav.landingFlag && nav.phase == FINISH_STRAIGHT
        && nav.finishDistance >= FINISH_DISTANCE_MM) {
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
            if (nav.phase == APPROACH_RAMP && nav.motion == ROUTE_DRIVE) {
                nav.routeHeading = netAngle;
            }
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
    /* 绕障直行也可能驶上坡道；坡度确认优先于本周期的碰撞倒车。 */
    if (!cliff && (nav.motion == ROUTE_DRIVE || nav.motion == OBSTACLE_CLEAR)
        && nav.phase != FINISH_STRAIGHT && nav.phase != FINISHED) {
        updateRoute(&nav, accelAxes, netAngle);
        if (nav.phase == FINISHED) {
            outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
                pRightWheelSpeed, pLeftWheelSpeed);
            return;
        }
    }

    /* 【入口确认】轮速保持零，使用碰撞之后新收集的完整 8 次读数。
     * updateRoute 收满 8 次且达到 6 票时切到 CLIMB_RAMP，并通过 resetVotes 退出确认。
     * 收满后仍未确认，就恢复地面处理；不因持续碰撞反复重开确认窗口。
     */
    if (nav.confirmingUphill) {
        if (nav.pitchCount < PITCH_WINDOW_SAMPLE_COUNT) {
            outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
                pRightWheelSpeed, pLeftWheelSpeed);
            return;
        }
        resetVotes(&nav);
        if (bumper) { beginObstacle(&nav, sensors, netDistance, netAngle); }
    }
    if (nav.motion == CLIFF_BACKUP
        && backupDistanceReached(netDistance, nav.maneuverDistance, CLIFF_BACKUP_DISTANCE_MM)) {
        nav.maneuverAngle = netAngle;
        nav.motion = CLIFF_TURN;
    }
    else if (nav.motion == CLIFF_TURN
        && abs(signedAngleError(netAngle, nav.maneuverAngle)) >= CLIFF_TURN_ANGLE_DEG) {
        if (cliff) { beginCliff(&nav, sensors, netDistance); }
        else {
            nav.motion = ROUTE_DRIVE;
            nav.routeHeading = netAngle; /* 不回到刚才朝向悬崖的旧航向 */
            resetCliffVotes(&nav);
        }
    }

    /* 只在前进遇到疑似上坡碰撞时暂停确认；单次倾角不直接关闭碰撞避障。
     * 悬崖已经抢占动作；地面倒车、转向仍按原流程完成。
     */
    if (nav.phase == APPROACH_RAMP && bumper
        && (nav.motion == ROUTE_DRIVE || nav.motion == OBSTACLE_CLEAR)
        && classifySlope(accelAxes, false) == SAMPLE_UPHILL) {
        resetVotes(&nav);
        nav.confirmingUphill = true;
        nav.headingControlActive = false;
        outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
            pRightWheelSpeed, pLeftWheelSpeed);
        return;
    }

    /* 【地面避障】上坡、坡顶、下坡完全跳过 bumper 处理。 */
    if (onGround(nav.phase) && nav.motion != CLIFF_BACKUP && nav.motion != CLIFF_TURN) {
        turnCompleted = nav.motion == OBSTACLE_TURN
            && abs(signedAngleError(netAngle, nav.maneuverAngle)) >= OBSTACLE_TURN_ANGLE_DEG;
        if (bumper && ((nav.motion != OBSTACLE_BACKUP && nav.motion != OBSTACLE_TURN)
                || (turnCompleted && nav.phase != APPROACH_RAMP))) {
            beginObstacle(&nav, sensors, netDistance, netAngle);
        }
        else if (nav.motion == OBSTACLE_BACKUP
            && backupDistanceReached(netDistance, nav.maneuverDistance, OBSTACLE_BACKUP_DISTANCE_MM)) {
            nav.maneuverAngle = netAngle;
            nav.motion = OBSTACLE_TURN;
        }
        else if (turnCompleted) {
            nav.maneuverDistance = netDistance;
            nav.clearHeading = netAngle;
            nav.motion = OBSTACLE_CLEAR;
        }
        else if (nav.motion == OBSTACLE_CLEAR
            && (nav.phase == APPROACH_RAMP
                ? backupDistanceReached(netDistance, nav.maneuverDistance, OBSTACLE_CLEAR_DISTANCE_MM)
                : (int64_t)netDistance - nav.maneuverDistance >= OBSTACLE_CLEAR_DISTANCE_MM)) {
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
            if (nav.yCooldownMs > 0) {
                if (nav.yCooldownMs > dtMs) { nav.yCooldownMs -= dtMs; }
                else { nav.yCooldownMs = 0; }
                correction = 0;
            }
            else {
                correction = updateY(&nav, accelAxes.y);
                if (correction != 0) {
                    nav.yCorrectionDurationMs += dtMs;
                    if (nav.yCorrectionDurationMs >= Y_CORRECTION_MAX_DURATION_MS) {
                        /* 持续纠偏满 2 秒：强制退出差速并重置采样窗口，进入平稳直行冷却 */
                        correction = 0;
                        nav.yIndex = nav.yCount = 0;
                        nav.yCorrectionDurationMs = 0;
                        nav.yCooldownMs = Y_CORRECTION_COOLDOWN_MS;
                    }
                }
                else {
                    /* 自然退出差速：Y轴回正，重置纠偏持续时间 */
                    nav.yCorrectionDurationMs = 0;
                }
            }
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
        if (nav.phase == APPROACH_RAMP) {
            /* 参考文件：按避障转向的反方向、固定轮速回到原路线航向。 */
            speed = limitSpeed(OBSTACLE_TURN_SPEED_MM_S, maxWheelSpeed);
            left = nav.turnRight ? -speed : speed;
            right = -left;
            break;
        }
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
    maintainApproachHeading(&nav, netAngle, maxWheelSpeed, &left, &right);
    outputSpeeds(&nav, left, right, maxWheelSpeed, netDistance,
        pRightWheelSpeed, pLeftWheelSpeed);
}
