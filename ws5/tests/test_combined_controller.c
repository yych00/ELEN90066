/* 离线传感器序列测试：只检查公开接口输出，不替代机器人实机验证。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../KobukiNavigationStateChart_combined.c"

static KobukiSensors_t input;
static accelerometer_t acceleration;
static int16_t leftSpeed, rightSpeed;
static int16_t maximum = 200;
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
    expect(maximum < 200 ? maximum : 200, maximum < 200 ? maximum : 200);
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

int main(int argc, char **argv)
{
    int i;
    int leftTurn;
    int32_t angle;
    double heading;
    assert(argc >= 2);
    if (argc >= 3) { simulator = strcmp(argv[2], "simulator") == 0; }
    start(0);

    if (strcmp(argv[1], "route_stop") == 0) {
        /* 初始地面不能停车，坡顶也不能作为终点。 */
        terrain(0, 0); samples(20, 0, 0); expect(200, 200);
        climb();
        terrain(0, 0); samples(8, 0, 0); expect(150, 150);
        terrain(-15, 0); samples(8, 0, 0); expect(100, 100);
        terrain(0, 0); samples(7, 0, 0); expect(100, 100);
        tick(0, 0); expect(200, 200);
        tick(199, 0); expect(200, 200);
        tick(200, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "no_flat_top") == 0) {
        finishStraight(); expect(200, 200);
        tick(200, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "vote_tolerance") == 0) {
        descend();
        terrain(0, 0); samples(6, 0, 0);
        terrain(0, 0.258819); samples(2, 0, 0); expect(100, 100);
        terrain(0, 0); samples(7, 0, 0); expect(200, 200);
    }
    else if (strcmp(argv[1], "bumper_on_slope") == 0) {
        climb();
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); expect(200, 200);
        terrain(0, 0); samples(8, 0, 0); expect(150, 150);
        terrain(-15, 0); samples(8, 0, 0); expect(100, 100);
        /* 回到地面后重新启用 bumper。 */
        terrain(0, 0); samples(8, 0, 0); expect(-120, -120);
    }
    else if (strcmp(argv[1], "cross_slope") == 0) {
        descend();
        terrain(0, 0.258819); samples(24, 0, 0);
        assert(leftSpeed > 0 && rightSpeed > 0);
        /* 横向倾斜不能启动 200 mm 终点计距。 */
        tick(1000, 0); assert(leftSpeed > 0 && rightSpeed > 0);
        terrain(0, 0); samples(8, 1000, 0); expect(200, 200);
        tick(1199, 0); expect(200, 200);
        tick(1200, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "invalid_acceleration") == 0) {
        descend();
        acceleration.x = acceleration.y = acceleration.z = 0;
        samples(20, 0, 0); expect(100, 100);
    }
    else if (strcmp(argv[1], "y_votes") == 0) {
        climb();
        terrain(15, 0.02); tick(0, 0);
        terrain(15, 0); tick(0, 0);
        terrain(15, 0.02); tick(0, 0);
        terrain(15, 0); tick(0, 0); expect(200, 200);
        terrain(15, 0.02); tick(0, 0); expect(200, 150);
        terrain(-15, 0); samples(8, 0, 0);
        terrain(-15, 0.02); samples(5, 0, 0); expect(75, 100);
        terrain(-15, -0.02); samples(5, 0, 0); expect(100, 75);
    }
    else if (strcmp(argv[1], "pause_votes") == 0) {
        descend();
        terrain(0, 0); samples(6, 0, 0); expect(100, 100);
        pauseAt(0, 0);
        resumeAt(0, 0); expect(100, 100);
        samples(6, 0, 0); expect(100, 100);
        tick(0, 0); expect(200, 200);
    }
    else if (strcmp(argv[1], "finish_pause") == 0) {
        finishStraight(); tick(100, 0); expect(200, 200);
        pauseAt(100, 0);
        tick(1000, 0); expect(0, 0);
        resumeAt(1400, 0); expect(200, 200);
        tick(1499, 0); expect(200, 200);
        tick(1500, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "finish_obstacle") == 0) {
        finishStraight(); tick(100, 0);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(100, 0); expect(-120, -120);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-20, 0); expect(110, -110);
        tick(-20, -80); expect(200, 200);
        tick(280, -80); expect(-110, 110);
        tick(280, 0); expect(200, 200);
        tick(379, 0); expect(200, 200);
        tick(380, 0); expect(0, 0);
    }
    else if (strcmp(argv[1], "finish_cliff") == 0) {
        finishStraight(); tick(100, 0);
        input.cliffLeft = true; tick(100, 0); expect(-50, -80);
        tick(40, 0); expect(80, -80);
        input.cliffLeft = false; tick(40, -30); expect(200, 200);
        tick(139, -30); expect(200, 200);
        tick(140, -30); expect(0, 0);
    }
    else if (strcmp(argv[1], "persistent_cliff") == 0) {
        descend();
        input.cliffLeft = true; tick(0, 0); expect(-50, -80);
        tick(-60, 0); expect(80, -80);
        tick(-60, -30); expect(-50, -80);
        tick(-120, -30); expect(80, -80);
        input.cliffLeft = false; tick(-120, -60); expect(100, 100);
        terrain(0, 0); samples(7, -120, -60); expect(100, 100);
        tick(-120, -60); expect(200, 200);
    }
    else if (strcmp(argv[1], "cliff_interrupts_obstacle") == 0) {
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); expect(-120, -120);
        input.cliffLeft = true; tick(-20, 0); expect(-50, -80);
        tick(-80, 0); expect(80, -80);
        input.cliffLeft = false;
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-80, -30); expect(200, 200);
    }
    else if (strcmp(argv[1], "resume_bumper") == 0) {
        pauseAt(0, 0);
        input.bumps_wheelDrops.bumpCenter = true;
        resumeAt(0, 0); expect(-120, -120);
    }
    else if (strcmp(argv[1], "persistent_bumper") == 0) {
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); tick(-120, 0); expect(110, -110);
        tick(-120, -80); expect(-120, -120);
        tick(-239, -80); expect(-120, -120);
    }
    else if (strcmp(argv[1], "obstacle_left") == 0
        || strcmp(argv[1], "obstacle_right") == 0
        || strcmp(argv[1], "obstacle_center") == 0) {
        leftTurn = strcmp(argv[1], "obstacle_right") == 0;
        input.bumps_wheelDrops.bumpRight = leftTurn;
        input.bumps_wheelDrops.bumpLeft = strcmp(argv[1], "obstacle_left") == 0;
        input.bumps_wheelDrops.bumpCenter = strcmp(argv[1], "obstacle_center") == 0;
        tick(0, 0); expect(-120, -120);
        memset(&input, 0, sizeof(input));
        tick(-120, 0); expect(leftTurn ? -110 : 110, leftTurn ? 110 : -110);
        angle = leftTurn ? 80 : -80;
        tick(-120, angle); expect(200, 200);
        tick(180, angle); expect(leftTurn ? 110 : -110, leftTurn ? -110 : 110);
        tick(180, 0); expect(200, 200);
    }
    else if (strcmp(argv[1], "overshoot") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80); tick(180, -80);
        tick(180, -6); expect(-20, 20);
        tick(180, 6); expect(20, -20);
        /* 简单差速运动模型：验证回正输出可以收敛，而不是继续转圈。 */
        heading = 6;
        for (i = 0; i < 300 && leftSpeed != rightSpeed; i++) {
            heading += (rightSpeed - leftSpeed) / 230.0 * RAD_TO_DEG * 0.064;
            angle = (int32_t)(heading >= 0 ? heading + 0.5 : heading - 0.5);
            tick(180, angle);
        }
        expect(200, 200);
        assert(i < 300);
    }
    else if (strcmp(argv[1], "angle_wrap") == 0) {
        start(350);
        input.bumps_wheelDrops.bumpRight = true; tick(0, 350);
        input.bumps_wheelDrops.bumpRight = false;
        tick(-120, 350); expect(-110, 110);
        tick(-120, 70); expect(200, 200);
        tick(180, 70); expect(110, -110);
        tick(180, -10); expect(200, 200);
    }
    else if (strcmp(argv[1], "ramp_during_clear") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        terrain(15, 0); samples(8, -120, -80);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(-120, -80); expect(200, 200);
        terrain(-15, 0); samples(8, -120, -80); expect(100, 100);
    }
    else if (strcmp(argv[1], "finished_latch") == 0) {
        finishStraight(); tick(200, 0); expect(0, 0);
        input.buttons.B0 = true;
        input.cliffLeft = input.bumps_wheelDrops.bumpLeft = true;
        terrain(-15, 0); tick(500, 0); expect(0, 0);
        input.buttons.B0 = false; tick(500, 0); expect(0, 0);
        input.buttons.B0 = true; tick(500, 0); expect(0, 0);
        input.buttons.B0 = false; tick(500, 0); expect(0, 0);
        start(0); expect(200, 200);
    }
    else if (strcmp(argv[1], "reset_held_button") == 0) {
        input.buttons.B0 = input.buttons.B1 = true;
        tick(0, 0); expect(0, 0);
        input.buttons.B1 = false;
        tick(0, 0); expect(0, 0);
        input.buttons.B0 = false;
        tick(0, 0); expect(0, 0);
        tick(0, 0); expect(0, 0);
        resumeAt(0, 0); expect(200, 200);
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
