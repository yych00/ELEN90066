/*
 * WS5 + 地面避障完整控制器：版本 1.1，combined.c 保留 1.0 对照。
 * 修复：绕行可确认上坡；限速保留 Y 差速；回正越过目标后反向；终点计距支持回绕。
 * 加速度提供单位、轴映射及安装偏角配置，未知实机输入时保留默认 g 单位。
 * Eclipse 可查看 kobukiDebug：阶段、传感器、投票、计距、轮速；不依赖 LED/myRIO 头文件。
 * 1.0 实机反馈为下坡不停车、Y 纠偏不生效；本版修复可复现代码问题，仍需实机验收。
 * 接近坡道的碰撞避障采用 new5_downhill_stop_jzcv2.c 的动作与航向控制。
 * 保留原两倍轮速、坡道与终点流程；加速度单位 g，距离 mm，轮速 mm/s。
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
#define Y_ALIGNMENT_THRESHOLD_G         0.006 /* 更灵敏：进入/退出共用五次窗口、三票门槛 */
#define Y_FILTER_SAMPLE_COUNT            5
#define Y_REQUIRED_SAMPLE_COUNT          3

/* 【实机输入配置】轴编号 0=X、1=Y、2=Z；配置映射后的 X 朝前、Y 横向、Z 朝上。
 * 默认输入单位 g；若调用方传 m/s^2，把 ACCEL_UNITS_PER_G 设为 9.80665。
 * 不自动猜单位/方向，不在坡上自动校准。平放静止时测安装偏角后再设置补偿。
 * 先消除横滚，再消除俯仰；默认偏角 0。所有配置均可在工程 -D 宏中覆盖。
 */
#ifndef ACCEL_UNITS_PER_G
#define ACCEL_UNITS_PER_G                1.0
#endif
#ifndef ACCEL_FORWARD_AXIS
#define ACCEL_FORWARD_AXIS               0
#endif
#ifndef ACCEL_SIDE_AXIS
#define ACCEL_SIDE_AXIS                  1
#endif
#ifndef ACCEL_VERTICAL_AXIS
#define ACCEL_VERTICAL_AXIS              2
#endif
#ifndef ACCEL_FORWARD_SIGN
#define ACCEL_FORWARD_SIGN               1
#endif
#ifndef ACCEL_SIDE_SIGN
#define ACCEL_SIDE_SIGN                  1
#endif
#ifndef ACCEL_VERTICAL_SIGN
#define ACCEL_VERTICAL_SIGN              1
#endif
#ifndef ACCEL_MOUNT_ROLL_DEG
#define ACCEL_MOUNT_ROLL_DEG              0.0
#endif
#ifndef ACCEL_MOUNT_PITCH_DEG
#define ACCEL_MOUNT_PITCH_DEG             0.0
#endif
/* 正常前进时净里程应递增；仅实测确认反向后才设为 -1。距离接口仍为 mm。 */
#ifndef DISTANCE_FORWARD_SIGN
#define DISTANCE_FORWARD_SIGN            1
#endif

/* 【路线轮速】先按阶段选择，再应用 Y 纠偏和固定轮速补偿。 */
#define APPROACH_SPEED_MM_S              400
#define CLIMB_SPEED_MM_S                 400
#define TOP_SPEED_MM_S                   300
#define DESCEND_SPEED_MM_S               200
#define FINISH_SPEED_MM_S                400
#ifndef FINISH_DISTANCE_MM
#define FINISH_DISTANCE_MM               200  /* 下坡确认平地后，正常直行的距离 */
#endif
#define UPHILL_ALIGN_OUTER_SPEED_MM_S    400
#define UPHILL_ALIGN_INNER_SPEED_MM_S    300
#define DOWNHILL_ALIGN_OUTER_SPEED_MM_S  200
#define DOWNHILL_ALIGN_INNER_SPEED_MM_S  150

/* 【地面碰撞避障】左撞右转、右撞左转、中央/双侧碰撞交替转向。 */
#define OBSTACLE_BACKUP_DISTANCE_MM      120
#define OBSTACLE_CLEAR_DISTANCE_MM       300
#define OBSTACLE_TURN_ANGLE_DEG          80
#define OBSTACLE_REVERSE_SPEED_MM_S      240
#define OBSTACLE_TURN_SPEED_MM_S         220
#define HEADING_TOLERANCE_DEG            5
#define REORIENT_MIN_SPEED_MM_S          40
#define REORIENT_GAIN_MM_S_PER_DEG       6

/* 【前半段航向保持】移植参考文件的阻尼、积分和每周期修正限幅参数。 */
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
#define CLIFF_BACKUP_FAST_MM_S           160
#define CLIFF_BACKUP_SLOW_MM_S           100
#define CLIFF_TURN_SPEED_MM_S            160
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
#if ACCEL_FORWARD_AXIS < 0 || ACCEL_FORWARD_AXIS > 2 || ACCEL_SIDE_AXIS < 0 || ACCEL_SIDE_AXIS > 2 || ACCEL_VERTICAL_AXIS < 0 || ACCEL_VERTICAL_AXIS > 2
#error Invalid_accelerometer_axis
#endif
#if ACCEL_FORWARD_AXIS == ACCEL_SIDE_AXIS || ACCEL_FORWARD_AXIS == ACCEL_VERTICAL_AXIS || ACCEL_SIDE_AXIS == ACCEL_VERTICAL_AXIS
#error Accelerometer_axes_must_be_distinct
#endif
#if (ACCEL_FORWARD_SIGN != 1 && ACCEL_FORWARD_SIGN != -1) || (ACCEL_SIDE_SIGN != 1 && ACCEL_SIDE_SIGN != -1) || (ACCEL_VERTICAL_SIGN != 1 && ACCEL_VERTICAL_SIGN != -1) || (DISTANCE_FORWARD_SIGN != 1 && DISTANCE_FORWARD_SIGN != -1)
#error Invalid_input_direction
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
    slopeSample_t pitchSamples[PITCH_WINDOW_SAMPLE_COUNT];
    int pitchIndex;
    int pitchCount;
    int ySamples[Y_FILTER_SAMPLE_COUNT];
    int yIndex;
    int yCount;
} navigation_t;

/* 【只读调试快照】在 Eclipse Expressions 中展开 kobukiDebug。
 * phase: 0接近、1上坡、2坡顶、3下坡、4终点直行、5停车。
 * motion: 0直行、1碰撞倒车、2转向、3绕行、4回正、5悬崖倒车、6悬崖转向。
 * invalidAccelSamples/stillDistanceSamples 是连续调用次数，不是毫秒或超时。
 */
typedef struct {
    int phase, motion, button;
    double rawX, rawY, rawZ;
    double xG, yG, zG, magnitudeG, pitchDeg, tiltDeg;
    bool accelValid;
    int slopeSample, upVotes, downVotes, levelVotes, pitchCount;
    int yPositiveVotes, yNegativeVotes, yCount, yCorrection;
    int invalidAccelSamples, stillDistanceSamples;
    int32_t distance, finishDistance;
    int64_t forwardDelta;
    bool countingFinish, finished;
    int16_t maxWheelSpeed, leftWheelSpeed, rightWheelSpeed;
} kobukiDebug_t;
volatile kobukiDebug_t kobukiDebug;

/* 验证、换算和安装补偿只做一次，坡度与 Y 判断共用同一组有效输入。 */
static accelerometer_t prepareAcceleration(const accelerometer_t raw)
{
    accelerometer_t result;
    double axes[3], x, y, z, rotatedY, rotatedZ;
    double roll = ACCEL_MOUNT_ROLL_DEG / RAD_TO_DEG;
    double pitch = ACCEL_MOUNT_PITCH_DEG / RAD_TO_DEG;
    double magnitude;
    memset(&result, 0, sizeof(result));
    kobukiDebug.rawX = raw.x; kobukiDebug.rawY = raw.y; kobukiDebug.rawZ = raw.z;
    kobukiDebug.accelValid = false;
    kobukiDebug.xG = kobukiDebug.yG = kobukiDebug.zG = 0.0;
    kobukiDebug.pitchDeg = kobukiDebug.tiltDeg = kobukiDebug.magnitudeG = 0.0;
    if (!(ACCEL_UNITS_PER_G > 0.0)) { return result; }
    axes[0] = raw.x / ACCEL_UNITS_PER_G;
    axes[1] = raw.y / ACCEL_UNITS_PER_G;
    axes[2] = raw.z / ACCEL_UNITS_PER_G;
    x = axes[ACCEL_FORWARD_AXIS] * ACCEL_FORWARD_SIGN;
    y = axes[ACCEL_SIDE_AXIS] * ACCEL_SIDE_SIGN;
    z = axes[ACCEL_VERTICAL_AXIS] * ACCEL_VERTICAL_SIGN;
    /* 比较形式同时拒绝 NaN/无穷，避免平方溢出；异常读数不参与 Y 投票。 */
    if (!(fabs(x) <= 1.5 && fabs(y) <= 1.5 && fabs(z) <= 1.5)) { return result; }
    magnitude = sqrt(x * x + y * y + z * z);
    kobukiDebug.magnitudeG = magnitude;
    if (!(magnitude >= 0.5 && magnitude <= 1.5)) { return result; }
    rotatedY = y * cos(roll) - z * sin(roll);
    rotatedZ = y * sin(roll) + z * cos(roll);
    result.x = x * cos(pitch) - rotatedZ * sin(pitch);
    result.y = rotatedY;
    result.z = x * sin(pitch) + rotatedZ * cos(pitch);
    if (!(fabs(result.x) <= 1.5 && fabs(result.y) <= 1.5 && fabs(result.z) <= 1.5)) {
        memset(&result, 0, sizeof(result));
        return result;
    }
    kobukiDebug.xG = result.x; kobukiDebug.yG = result.y; kobukiDebug.zG = result.z;
    kobukiDebug.pitchDeg = atan2(result.x, sqrt(result.y * result.y + result.z * result.z)) * RAD_TO_DEG;
    kobukiDebug.tiltDeg = atan2(sqrt(result.x * result.x + result.y * result.y), fabs(result.z)) * RAD_TO_DEG;
    kobukiDebug.accelValid = true;
    return result;
}

/* 成对缩放保留差速比例，例如上坡 400/300 在上限 200 时输出 200/150。 */
static void limitWheelPair(int16_t * const left, int16_t * const right, const int16_t maximum)
{
    int peak = abs(*left) > abs(*right) ? abs(*left) : abs(*right);
    if (maximum <= 0) { *left = *right = 0; return; }
    if (peak > maximum) {
        *left = (int16_t)((int32_t)*left * maximum / peak);
        *right = (int16_t)((int32_t)*right * maximum / peak);
    }
}

/* 净里程是 int32 累计 mm；支持正常的 32 位回绕，单次真实变化须小于半量程。 */
static int64_t forwardDistanceDelta(const int32_t current, const int32_t previous)
{
    int64_t delta = (int64_t)current - previous;
    if (delta > 2147483647LL) { delta -= 4294967296LL; }
    else if (delta < -2147483648LL) { delta += 4294967296LL; }
    return delta * DISTANCE_FORWARD_SIGN;
}

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

/* 【Y 差速纠偏】五次中三次同方向超过 ±0.006 g 时进入；
 * 两个方向都不足三票时退出，反方向达到三票时直接切换。
 * 进入、退出共用同一个窗口；只覆盖正常上、下坡的直行轮速。
 */
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
    resetVotes(nav);
}

/* 【前半段直行/绕行纠偏】正常直行锁定路线航向，绕行锁定转向完成时的航向。
 * 保留配置的基础速度；达到调用方轮速上限时，两轮同时减去超出量，保持差速。
 * 和参考文件一样，KD 与积分按调用次数计算，调用周期变化会影响实际纠偏效果。
 */
static void maintainApproachHeading(navigation_t * const nav, const int32_t angle,
    const int16_t maximum, int16_t * const left, int16_t * const right)
{
    int32_t target;
    double error, yawStep = 0.0, correction, correctionLimit, baseSpeed;
    double requestedLeft, requestedRight, excess;
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
    excess = (requestedLeft > requestedRight ? requestedLeft : requestedRight) - maximum;
    if (excess > 0.0) { requestedLeft -= excess; requestedRight -= excess; }
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
    int i;
    *pLeft = trimWheelSpeed(left, WHEEL_SPEED_TRIM < 0 ? -WHEEL_SPEED_TRIM : 0, 32767);
    *pRight = trimWheelSpeed(right, WHEEL_SPEED_TRIM > 0 ? WHEEL_SPEED_TRIM : 0, 32767);
    limitWheelPair(pLeft, pRight, maximum);
    nav->lastDistance = distance;
    nav->countFinishDistance = nav->phase == FINISH_STRAIGHT
        && nav->button == RUNNING && nav->motion == ROUTE_DRIVE
        && *pLeft > 0 && *pRight > 0;
    if (nav->button != RUNNING) { nav->headingControlActive = false; }
    kobukiDebug.phase = nav->phase; kobukiDebug.motion = nav->motion; kobukiDebug.button = nav->button;
    kobukiDebug.pitchCount = nav->pitchCount; kobukiDebug.yCount = nav->yCount;
    kobukiDebug.upVotes = kobukiDebug.downVotes = kobukiDebug.levelVotes = 0;
    kobukiDebug.yPositiveVotes = kobukiDebug.yNegativeVotes = 0;
    for (i = 0; i < nav->pitchCount; i++) {
        if (nav->pitchSamples[i] == SAMPLE_UPHILL) { kobukiDebug.upVotes++; }
        if (nav->pitchSamples[i] == SAMPLE_DOWNHILL) { kobukiDebug.downVotes++; }
        if (nav->pitchSamples[i] == SAMPLE_LEVEL) { kobukiDebug.levelVotes++; }
    }
    for (i = 0; i < nav->yCount; i++) {
        if (nav->ySamples[i] > 0) { kobukiDebug.yPositiveVotes++; }
        if (nav->ySamples[i] < 0) { kobukiDebug.yNegativeVotes++; }
    }
    kobukiDebug.distance = distance; kobukiDebug.finishDistance = nav->finishDistance;
    kobukiDebug.countingFinish = nav->countFinishDistance;
    kobukiDebug.finished = nav->phase == FINISHED;
    kobukiDebug.maxWheelSpeed = maximum;
    kobukiDebug.leftWheelSpeed = *pLeft; kobukiDebug.rightWheelSpeed = *pRight;
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
    accelerometer_t calibratedAccel;
    (void)isSimulator;

    calibratedAccel = prepareAcceleration(accelAxes);
    kobukiDebug.slopeSample = classifySlope(calibratedAccel);
    kobukiDebug.yCorrection = 0;
    kobukiDebug.forwardDelta = 0;
    if (kobukiDebug.accelValid) { kobukiDebug.invalidAccelSamples = 0; }
    else if (kobukiDebug.invalidAccelSamples < 2147483647) { kobukiDebug.invalidAccelSamples++; }

    /* B1 重置后仍为停车等待状态，必须重新按下并松开 B0。 */
    if (!nav.initialized || sensors.buttons.B1) {
        resetRun(&nav, netDistance, netAngle);
        kobukiDebug.stillDistanceSamples = 0;
        kobukiDebug.invalidAccelSamples = kobukiDebug.accelValid ? 0 : 1;
        /* 重置时若 B0 已按住，先等它释放，避免把旧按键当成新启动。 */
        if (sensors.buttons.B0) { nav.button = WAIT_PAUSE_RELEASE; }
        outputSpeeds(&nav, 0, 0, maxWheelSpeed, netDistance,
            pRightWheelSpeed, pLeftWheelSpeed);
        return;
    }

    /* 用上一周期的命令决定计距：只加前进距离，不把倒车取绝对值。 */
    if (nav.countFinishDistance) {
        delta = forwardDistanceDelta(netDistance, nav.lastDistance);
        kobukiDebug.forwardDelta = delta;
        if (delta == 0) {
            if (kobukiDebug.stillDistanceSamples < 2147483647) { kobukiDebug.stillDistanceSamples++; }
        }
        else { kobukiDebug.stillDistanceSamples = 0; }
        if (delta > 0) {
            if (delta >= FINISH_DISTANCE_MM - nav.finishDistance) {
                nav.finishDistance = FINISH_DISTANCE_MM;
            }
            else { nav.finishDistance += (int32_t)delta; }
        }
    }
    else { kobukiDebug.stillDistanceSamples = 0; }
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
    /* 绕障直行也可能驶上坡；确认后立即结束地面避障，交给坡道控制。
     * 倒车、转向及悬崖动作不采样，防止把运动加速度误当作上/下坡。
     */
    if (!cliff && (nav.motion == ROUTE_DRIVE || nav.motion == OBSTACLE_CLEAR)
        && nav.phase != FINISH_STRAIGHT) {
        updateRoute(&nav, calibratedAccel, netAngle);
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
            resetVotes(&nav);
        }
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
                : forwardDistanceDelta(netDistance, nav.maneuverDistance) >= OBSTACLE_CLEAR_DISTANCE_MM)) {
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
            correction = updateY(&nav, kobukiDebug.accelValid ? calibratedAccel.y : 0.0);
            if (!kobukiDebug.accelValid) { correction = 0; }
            kobukiDebug.yCorrection = correction;
            if (correction != 0) {
                if (nav.phase == CLIMB_RAMP) {
                    left = correction > 0 ? UPHILL_ALIGN_OUTER_SPEED_MM_S : UPHILL_ALIGN_INNER_SPEED_MM_S;
                    right = correction > 0 ? UPHILL_ALIGN_INNER_SPEED_MM_S : UPHILL_ALIGN_OUTER_SPEED_MM_S;
                }
                else {
                    left = correction > 0 ? DOWNHILL_ALIGN_INNER_SPEED_MM_S : DOWNHILL_ALIGN_OUTER_SPEED_MM_S;
                    right = correction > 0 ? DOWNHILL_ALIGN_OUTER_SPEED_MM_S : DOWNHILL_ALIGN_INNER_SPEED_MM_S;
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
        /* 两个地面阶段统一按剩余误差回正；越过目标后反向，避免原地转整圈。 */
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
