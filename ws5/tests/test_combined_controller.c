/* 离线传感器序列测试：只检查公开接口输出，不替代机器人实机验证。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifndef COMBINED_CONTROLLER_SOURCE
#define COMBINED_CONTROLLER_SOURCE "../KobukiNavigationStateChart_combined.c"
#endif
#include COMBINED_CONTROLLER_SOURCE

static KobukiSensors_t input;
static accelerometer_t acceleration;
static int16_t leftSpeed, rightSpeed;
static int16_t maximum = 400;
static bool simulator = false;

static void tick(const int32_t distance, const int32_t angle)
{
    KobukiNavigationStatechart(maximum, distance, angle, input, acceleration,
        &rightSpeed, &leftSpeed, simulator);
    assert(abs(leftSpeed) <= maximum);
    assert(abs(rightSpeed) <= maximum);
}

static void expect(const int16_t left, const int16_t right)
{
    if (leftSpeed != left || rightSpeed != right) {
        fprintf(stderr, "expected (%d,%d), got (%d,%d)\n", left, right, leftSpeed, rightSpeed);
        assert(0);
    }
}

static void terrain(const double pitch, const double y)
{
    double x = sin(pitch / RAD_TO_DEG);
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = sqrt(1 - x * x - y * y);
}

static void samples(const int count, const int32_t distance, const int32_t angle)
{
    int i;
    for (i = 0; i < count; i++) { tick(distance, angle); }
}

static void start(const int32_t angle)
{
    memset(&input, 0, sizeof(input));
    terrain(0, 0);
    input.buttons.B1 = true;
    tick(0, angle);
    expect(0, 0);
    input.buttons.B1 = false;
    input.buttons.B0 = true;
    tick(0, angle);
    expect(0, 0);
    input.buttons.B0 = false;
    tick(0, angle);
    expect(maximum < 400 ? maximum : 400, maximum < 400 ? maximum : 400);
}

static void climb(void)
{
    terrain(15, 0);
    /* 启动时已有一票平地，加七票上坡正好验证 8 中 7，并清空新阶段窗口。 */
    samples(7, 0, 0);
}

static void descend(void)
{
    climb();
    terrain(-15, 0);
    samples(8, 0, 0);
}

static void finishStraight(void)
{
    descend();
    terrain(0, 0);
    samples(8, 0, 0);
}

static void pauseAt(const int32_t distance, const int32_t angle)
{
    input.buttons.B0 = true;
    tick(distance, angle);
    expect(0, 0);
    input.buttons.B0 = false;
    tick(distance, angle);
    expect(0, 0);
}

static void resumeAt(const int32_t distance, const int32_t angle)
{
    input.buttons.B0 = true;
    tick(distance, angle);
    expect(0, 0);
    input.buttons.B0 = false;
    tick(distance, angle);
}

/* 同一倒车动作分别输入递增/递减里程，检查到距离阈值就转向。
 * 碰撞/悬崖信号保持或解除都不能反复重置倒车起点。
 */
static void checkBackupCounter(const bool cliff, const int32_t origin,
    const int direction, const bool releaseSensor)
{
    int travelled;
    int threshold = cliff ? 60 : 120;
    start(0);
    tick(origin, 0);
    input.cliffLeft = cliff;
    input.bumps_wheelDrops.bumpLeft = !cliff;
    tick(origin, 0);
    if (releaseSensor) {
        input.cliffLeft = input.bumps_wheelDrops.bumpLeft = false;
    }
    for (travelled = 0; travelled < threshold; travelled++) {
        tick(origin + direction * travelled, 0);
        expect(cliff ? -100 : -240, cliff ? -160 : -240);
    }
    tick(origin + direction * threshold, 0);
    expect(cliff ? 160 : 220, cliff ? -160 : -220);
}

/* 连续课程模型：轮速推动位置与里程，再由当前位置产生地形/碰撞读数。
 * 这是独立的理想差速模型，轮距 230 mm；不代表实机摩擦、惯性或制动性能。
 * 地面障碍位于 x=260..400、|y|<=70；上坡 700..1600，坡顶 1600..2600，
 * 下坡 2600..3500，后面为终点平面。所有传感器都通过公开接口送入。
 */
static void fullCourse(const char * const profile)
{
    int step;
    int courseSteps;
    int32_t odometer = 0;
    int32_t headingDegrees = 0;
    double x = 0, y = 0, heading = 0, distance = 0, elapsed = 0;
    double dt, leftActual, rightActual, linear, angular, slope;
    double finishStart = -1;
    double largestStep = 0;
    double maximumPeriod = 0;
    bool sawBackup = false, sawTurn = false, sawClimb = false;
    bool sawTop = false, sawDescent = false, stopped = false;
    bool limited = strcmp(profile, "course_capped200") == 0;
    bool drift = strcmp(profile, "course_wheel_drift") == 0;

    if (limited) { maximum = 200; start(0); }
    for (step = 0; step < 10000; step++) {
        dt = 0.064;
        if (strcmp(profile, "course_20ms") == 0) { dt = 0.020; }
        else if (strcmp(profile, "course_100ms") == 0) { dt = 0.100; }
        else if (strcmp(profile, "course_250ms") == 0) { dt = 0.250; }
        else if (strcmp(profile, "course_jitter") == 0) {
            dt = step % 3 == 0 ? 0.020 : (step % 3 == 1 ? 0.064 : 0.128);
        }
        if (dt > maximumPeriod) { maximumPeriod = dt; }
        leftActual = leftSpeed * (drift ? 0.99 : 1.0);
        rightActual = rightSpeed * (drift ? 1.01 : 1.0);
        linear = (leftActual + rightActual) / 2;
        angular = (rightActual - leftActual) / 230.0;
        x += linear * cos(heading + angular * dt / 2) * dt;
        y += linear * sin(heading + angular * dt / 2) * dt;
        heading += angular * dt;
        distance += linear * dt;
        if (linear * dt > largestStep) { largestStep = linear * dt; }
        odometer = (int32_t)(distance >= 0 ? distance + 0.5 : distance - 0.5);
        headingDegrees = (int32_t)(heading >= 0
            ? heading * RAD_TO_DEG + 0.5 : heading * RAD_TO_DEG - 0.5);

        memset(&input, 0, sizeof(input));
        elapsed += dt;
        input.TimeStamp = (uint16_t)(int64_t)(elapsed * 1000);
        input.bumps_wheelDrops.bumpCenter = x >= 260 && x <= 400 && fabs(y) <= 70;
        /* 爬坡已确认后的坡面故意拉高 bumper，验证不误入地面避障。 */
        if (x >= 1450 && x < 2700) { input.bumps_wheelDrops.bumpCenter = true; }
        slope = x >= 700 && x < 1600 ? 15.0
            : (x >= 2600 && x < 3500 ? -15.0 : 0.0);
        acceleration.x = sin(slope / RAD_TO_DEG) * cos(heading);
        acceleration.y = sin(slope / RAD_TO_DEG) * sin(heading);
        acceleration.z = cos(slope / RAD_TO_DEG);
        tick(odometer, headingDegrees);

        if (x < 700 && leftSpeed < 0 && rightSpeed < 0) { sawBackup = true; }
        if (x < 700 && ((leftSpeed < 0 && rightSpeed > 0)
            || (leftSpeed > 0 && rightSpeed < 0))) { sawTurn = true; }
        if (x >= 1450 && x < 1600 && leftSpeed > 0 && rightSpeed > 0) {
            sawClimb = true;
        }
        if (x >= 1600 && x < 2600 && leftSpeed == rightSpeed
            && leftSpeed == (limited ? 200 : 300)) { sawTop = true; }
        if (x >= 2600 && x < 3500 && leftSpeed > 0 && rightSpeed > 0
            && leftSpeed <= 200 && rightSpeed <= 200) { sawDescent = true; }
        if (!limited && x >= 3500 && leftSpeed == 400 && rightSpeed == 400
            && finishStart < 0) { finishStart = odometer; }
        if (x >= 3500 && leftSpeed == 0 && rightSpeed == 0) {
            stopped = true;
            break;
        }
    }
    assert(sawBackup && sawTurn && sawClimb && sawTop && sawDescent && stopped);
    courseSteps = step + 1;
    /* 不把最初平面/坡顶当终点，且到最终平面后不会无限继续跑。 */
    assert(x >= 3500 && x <= 3500 + 200 * maximumPeriod * 8 + 200 + largestStep + 10);
    if (!limited) {
        assert(finishStart >= 0);
        assert(odometer - finishStart >= 200);
        assert(odometer - finishStart <= 200 + largestStep + 1);
    }
    /* 连续按 B0、改变坡度、同时触发 bumper/cliff 都不能解除终点锁止。 */
    for (step = 0; step < 12; step++) {
        input.buttons.B0 = step % 2 == 0;
        input.cliffLeft = input.bumps_wheelDrops.bumpLeft = true;
        terrain(-15, 0);
        tick(odometer, headingDegrees);
        expect(0, 0);
    }
    printf("COURSE %s: avoid -> climb -> top -> descend -> stop; x=%.1f mm, steps=%d\n",
        profile, x, courseSteps);
}

int main(int argc, char **argv)
{
    int i;
    int leftTurn;
    int32_t angle;
    double heading;
    assert(argc >= 2);
    if (argc >= 3) { simulator = strcmp(argv[2], "simulator") == 0; }
    start(0);

    if (strncmp(argv[1], "course_", 7) == 0) {
        fullCourse(argv[1]);
    }
    else if (strcmp(argv[1], "route_stop") == 0) {
        /* 初始地面不能停车，坡顶也不能作为终点。 */
        terrain(0, 0); samples(20, 0, 0); expect(400, 400);
        climb();
        terrain(0, 0); samples(8, 0, 0); expect(300, 300);
        terrain(-15, 0); samples(8, 0, 0); expect(200, 200);
        terrain(0, 0); samples(7, 0, 0); expect(200, 200);
        tick(0, 0); expect(400, 400);
        tick(199, 0); expect(400, 400);
        tick(200, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "no_flat_top") == 0) {
        finishStraight(); expect(400, 400);
        tick(200, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "vote_tolerance") == 0) {
        descend();
        terrain(0, 0); samples(6, 0, 0);
        terrain(0, 0.258819); samples(2, 0, 0); expect(200, 200);
        terrain(0, 0); samples(7, 0, 0); expect(400, 400);
    }
    else if (strcmp(argv[1], "bumper_on_slope") == 0) {
        climb();
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); expect(400, 400);
        terrain(0, 0); samples(8, 0, 0); expect(300, 300);
        terrain(-15, 0); samples(8, 0, 0); expect(200, 200);
        /* 回到地面后重新启用 bumper。 */
        terrain(0, 0); samples(8, 0, 0); expect(-240, -240);
    }
    else if (strcmp(argv[1], "cross_slope") == 0) {
        descend();
        terrain(0, 0.258819); samples(24, 0, 0);
        assert(leftSpeed > 0 && rightSpeed > 0);
        /* 横向倾斜不能启动 200 mm 终点计距。 */
        tick(1000, 0); assert(leftSpeed > 0 && rightSpeed > 0);
        terrain(0, 0); samples(8, 1000, 0); expect(400, 400);
        tick(1199, 0); expect(400, 400);
        tick(1200, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "invalid_acceleration") == 0) {
        descend();
        acceleration.x = acceleration.y = acceleration.z = 0;
        samples(20, 0, 0); expect(200, 200);
    }
    else if (strcmp(argv[1], "y_votes") == 0) {
        climb();
        terrain(15, 0.02); tick(0, 0);
        terrain(15, 0); tick(0, 0);
        terrain(15, 0.02); tick(0, 0);
        terrain(15, 0); tick(0, 0); expect(400, 400);
        terrain(15, 0.02); tick(0, 0); expect(400, 300);
        terrain(-15, 0); samples(8, 0, 0);
        terrain(-15, 0.02); samples(5, 0, 0); expect(150, 200);
        terrain(-15, -0.02); samples(5, 0, 0); expect(200, 150);
    }
    else if (strcmp(argv[1], "pause_votes") == 0) {
        descend();
        terrain(0, 0); samples(6, 0, 0); expect(200, 200);
        pauseAt(0, 0);
        resumeAt(0, 0); expect(200, 200);
        samples(6, 0, 0); expect(200, 200);
        tick(0, 0); expect(400, 400);
    }
    else if (strcmp(argv[1], "finish_pause") == 0) {
        finishStraight(); tick(100, 0); expect(400, 400);
        pauseAt(100, 0);
        tick(1000, 0); expect(0, 0);
        resumeAt(1400, 0); expect(400, 400);
        tick(1499, 0); expect(400, 400);
        tick(1500, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "finish_obstacle") == 0) {
        finishStraight(); tick(100, 0);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(100, 0); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-20, 0); expect(220, -220);
        tick(-20, -80); expect(400, 400);
        tick(280, -80); expect(-220, 220);
        tick(280, 0); expect(400, 400);
        tick(379, 0); expect(400, 400);
        tick(380, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "finish_cliff") == 0) {
        finishStraight(); tick(100, 0);
        input.cliffLeft = true; tick(100, 0); expect(-100, -160);
        tick(40, 0); expect(160, -160);
        input.cliffLeft = false; tick(40, -30); expect(400, 400);
        tick(139, -30); expect(400, 400);
        tick(140, -30); expect(0, 0);
    }
    else if (strcmp(argv[1], "persistent_cliff") == 0) {
        descend();
        input.cliffLeft = true; tick(0, 0); expect(-100, -160);
        tick(-60, 0); expect(160, -160);
        tick(-60, -30); expect(-100, -160);
        tick(-120, -30); expect(160, -160);
        input.cliffLeft = false; tick(-120, -60); expect(200, 200);
        terrain(0, 0); samples(7, -120, -60); expect(200, 200);
        tick(-120, -60); expect(400, 400);
    }
    else if (strcmp(argv[1], "cliff_interrupts_obstacle") == 0) {
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); expect(-240, -240);
        input.cliffLeft = true; tick(-20, 0); expect(-100, -160);
        tick(-80, 0); expect(160, -160);
        input.cliffLeft = false;
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-80, -30); expect(400, 400);
    }
    else if (strcmp(argv[1], "resume_bumper") == 0) {
        pauseAt(0, 0);
        input.bumps_wheelDrops.bumpCenter = true;
        resumeAt(0, 0); expect(-240, -240);
    }
    else if (strcmp(argv[1], "persistent_bumper") == 0) {
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); tick(-120, 0); expect(220, -220);
        tick(-120, -80); expect(-240, -240);
        tick(-239, -80); expect(-240, -240);
    }
    else if (strcmp(argv[1], "backup_increasing_distance") == 0) {
        checkBackupCounter(false, 0, 1, false);
    }
    else if (strcmp(argv[1], "cliff_increasing_distance") == 0) {
        checkBackupCounter(true, 0, 1, false);
    }
    else if (strcmp(argv[1], "backup_start_offsets") == 0) {
        checkBackupCounter(false, 1000, 1, true);
        checkBackupCounter(false, -1000, 1, false);
        checkBackupCounter(false, 1000, -1, false);
        checkBackupCounter(false, -1000, -1, true);
    }
    else if (strcmp(argv[1], "cliff_start_offsets") == 0) {
        checkBackupCounter(true, 1000, 1, true);
        checkBackupCounter(true, -1000, 1, false);
        checkBackupCounter(true, 1000, -1, false);
        checkBackupCounter(true, -1000, -1, true);
    }
    else if (strcmp(argv[1], "backup_distance_overflow") == 0
        || strcmp(argv[1], "cliff_distance_overflow") == 0) {
        /* 两个 int32 读数相减时，先转为 int64，避免 abs(INT32_MIN) 溢出。 */
        input.cliffLeft = strcmp(argv[1], "cliff_distance_overflow") == 0;
        input.bumps_wheelDrops.bumpLeft = !input.cliffLeft;
        tick(0, 0);
        tick(INT32_MIN, 0);
        expect(input.cliffLeft ? 160 : 220, input.cliffLeft ? -160 : -220);
    }
    else if (strcmp(argv[1], "obstacle_left") == 0
        || strcmp(argv[1], "obstacle_right") == 0
        || strcmp(argv[1], "obstacle_center") == 0) {
        leftTurn = strcmp(argv[1], "obstacle_right") == 0;
        input.bumps_wheelDrops.bumpRight = leftTurn;
        input.bumps_wheelDrops.bumpLeft = strcmp(argv[1], "obstacle_left") == 0;
        input.bumps_wheelDrops.bumpCenter = strcmp(argv[1], "obstacle_center") == 0;
        tick(0, 0); expect(-240, -240);
        memset(&input, 0, sizeof(input));
        tick(-120, 0); expect(leftTurn ? -220 : 220, leftTurn ? 220 : -220);
        angle = leftTurn ? 80 : -80;
        tick(-120, angle); expect(400, 400);
        tick(180, angle); expect(leftTurn ? 220 : -220, leftTurn ? -220 : 220);
        tick(180, 0); expect(400, 400);
    }
    else if (strcmp(argv[1], "overshoot") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80); tick(180, -80);
        tick(180, -6); expect(-40, 40);
        tick(180, 6); expect(40, -40);
        /* 简单差速运动模型：验证回正输出可以收敛，而不是继续转圈。 */
        heading = 6;
        for (i = 0; i < 300 && leftSpeed != rightSpeed; i++) {
            heading += (rightSpeed - leftSpeed) / 230.0 * RAD_TO_DEG * 0.064;
            angle = (int32_t)(heading >= 0 ? heading + 0.5 : heading - 0.5);
            tick(180, angle);
        }
        expect(400, 400);
        assert(i < 300);
    }
    else if (strcmp(argv[1], "angle_wrap") == 0) {
        start(350);
        input.bumps_wheelDrops.bumpRight = true; tick(0, 350);
        input.bumps_wheelDrops.bumpRight = false;
        tick(-120, 350); expect(-220, 220);
        tick(-120, 70); expect(400, 400);
        tick(180, 70); expect(220, -220);
        tick(180, -10); expect(400, 400);
    }
    else if (strcmp(argv[1], "ramp_during_clear") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        terrain(15, 0); samples(8, -120, -80);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(-120, -80); expect(400, 400);
        terrain(-15, 0); samples(8, -120, -80); expect(200, 200);
    }
    else if (strcmp(argv[1], "finished_latch") == 0) {
        finishStraight(); tick(200, 0); expect(0, 0);
        input.buttons.B0 = true;
        input.cliffLeft = input.bumps_wheelDrops.bumpLeft = true;
        terrain(-15, 0); tick(500, 0); expect(0, 0);
        input.buttons.B0 = false; tick(500, 0); expect(0, 0);
        input.buttons.B0 = true; tick(500, 0); expect(0, 0);
        input.buttons.B0 = false; tick(500, 0); expect(0, 0);
        start(0); expect(400, 400);
    }
    else if (strcmp(argv[1], "reset_held_button") == 0) {
        input.buttons.B0 = input.buttons.B1 = true;
        tick(0, 0); expect(0, 0);
        input.buttons.B1 = false;
        tick(0, 0); expect(0, 0);
        input.buttons.B0 = false;
        tick(0, 0); expect(0, 0);
        tick(0, 0); expect(0, 0);
        resumeAt(0, 0); expect(400, 400);
    }
    else if (strcmp(argv[1], "speed_cap") == 0) {
        maximum = 40; start(0);
        finishStraight(); expect(40, 40);
        input.cliffLeft = true; tick(0, 0); expect(-40, -40);
        maximum = 0; tick(0, 0); expect(0, 0);
    }
    else { assert(!"unknown scenario"); }
    printf("PASS %s (%s)\n", argv[1], simulator ? "simulator" : "hardware interface");
    return 0;
}
