/* 验证参考文件的前半段碰撞避障行为，并复用原坡道/终点回归测试。 */
#define COMBINED_CONTROLLER_SOURCE "../KobukiNavigationStateChart_combined_debug.c"
#define main combinedRegressionMain
#include "test_combined_controller.c"
#undef main

/* 独立传感器模型：给定车体姿态，按安装旋转、接线和单位产生原始读数。 */
static void configuredTerrain(const double pitchDeg, const double sideG)
{
    double x = sin(pitchDeg / RAD_TO_DEG), y = sideG;
    double z = sqrt(1.0 - x * x - y * y);
    double pitchMount = ACCEL_MOUNT_PITCH_DEG / RAD_TO_DEG;
    double rollMount = ACCEL_MOUNT_ROLL_DEG / RAD_TO_DEG;
    double sensorX = x * cos(pitchMount) + z * sin(pitchMount);
    double intermediateZ = -x * sin(pitchMount) + z * cos(pitchMount);
    double sensorY = y * cos(rollMount) + intermediateZ * sin(rollMount);
    double sensorZ = -y * sin(rollMount) + intermediateZ * cos(rollMount);
    double axes[3];
    axes[ACCEL_FORWARD_AXIS] = sensorX * ACCEL_FORWARD_SIGN * ACCEL_UNITS_PER_G;
    axes[ACCEL_SIDE_AXIS] = sensorY * ACCEL_SIDE_SIGN * ACCEL_UNITS_PER_G;
    axes[ACCEL_VERTICAL_AXIS] = sensorZ * ACCEL_VERTICAL_SIGN * ACCEL_UNITS_PER_G;
    acceleration.x = axes[0]; acceleration.y = axes[1]; acceleration.z = axes[2];
}

static int repairScenario(const char * const name)
{
    int i;
    if (strcmp(name, "configured_course") == 0) {
        start(0);
        configuredTerrain(0, 0); samples(8, 0, 0);
        assert(kobukiDebug.accelValid && fabs(kobukiDebug.tiltDeg) < 0.001);
        configuredTerrain(15, 0); samples(8, 0, 0);
        assert(kobukiDebug.phase == CLIMB_RAMP);
        assert(fabs(kobukiDebug.pitchDeg - 15.0) < 0.001);
        configuredTerrain(15, 0.02); samples(5, 0, 0); expect(400, 300);
        configuredTerrain(15, -0.02); samples(5, 0, 0); expect(300, 400);
        configuredTerrain(0, 0); samples(8, 0, 0); expect(300, 300);
        assert(kobukiDebug.phase == DRIVE_ACROSS_TOP && !kobukiDebug.finished);
        configuredTerrain(-15, 0); samples(8, 0, 0);
        assert(kobukiDebug.phase == DESCEND_RAMP);
        configuredTerrain(-15, 0.02); samples(5, 0, 0); expect(150, 200);
        configuredTerrain(-15, -0.02); samples(5, 0, 0); expect(200, 150);
        configuredTerrain(0, 0); samples(8, 0, 0);
        assert(kobukiDebug.phase == FINISH_STRAIGHT && kobukiDebug.finishDistance == 0);
        tick(199 * DISTANCE_FORWARD_SIGN, 0); expect(400, 400);
        tick(200 * DISTANCE_FORWARD_SIGN, 0); expect(0, 0);
        assert(kobukiDebug.finished && kobukiDebug.finishDistance == 200);
        input.buttons.B0 = true; tick(201 * DISTANCE_FORWARD_SIGN, 0); expect(0, 0);
        input.buttons.B0 = false; tick(202 * DISTANCE_FORWARD_SIGN, 0); expect(0, 0);
        input.buttons.B1 = true; tick(202 * DISTANCE_FORWARD_SIGN, 0); expect(0, 0);
        assert(!kobukiDebug.finished && kobukiDebug.finishDistance == 0);
        /* 终点避障也必须遵守配置的里程方向，绕行距离不得算进最后 200 mm。 */
        start(0);
        configuredTerrain(15, 0); samples(8, 0, 0);
        configuredTerrain(-15, 0); samples(8, 0, 0);
        configuredTerrain(0, 0); samples(8, 0, 0);
        tick(50 * DISTANCE_FORWARD_SIGN, 0);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(50 * DISTANCE_FORWARD_SIGN, 0); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-70 * DISTANCE_FORWARD_SIGN, 0); expect(220, -220);
        tick(-70 * DISTANCE_FORWARD_SIGN, -80); expect(400, 400);
        tick(230 * DISTANCE_FORWARD_SIGN, -80); expect(-220, 220);
        tick(230 * DISTANCE_FORWARD_SIGN, 0); expect(400, 400);
        assert(kobukiDebug.finishDistance == 50);
        tick(379 * DISTANCE_FORWARD_SIGN, 0); expect(400, 400);
        tick(380 * DISTANCE_FORWARD_SIGN, 0); expect(0, 0);
    }
    else if (strcmp(name, "y_limited_pair") == 0) {
        maximum = 200; start(0); climb();
        terrain(15, 0.02); samples(5, 0, 0); expect(200, 150);
        terrain(15, -0.02); samples(5, 0, 0); expect(150, 200);
        maximum = 100;
        terrain(-15, 0); samples(8, 0, 0);
        terrain(-15, 0.02); samples(5, 0, 0); expect(75, 100);
        terrain(-15, -0.02); samples(5, 0, 0); expect(100, 75);
        assert(kobukiDebug.yCorrection == -1 && kobukiDebug.maxWheelSpeed == 100);
    }
    else if (strcmp(name, "y_sensitive_switching") == 0) {
        start(0); climb(); samples(5, 0, 0);
        /* 小于旧门槛的偏差：两票不触发，第三票开始纠偏。 */
        terrain(15, 0.008); samples(2, 0, 0); expect(400, 400);
        tick(0, 0); expect(400, 300);
        /* 中性样本让同方向票数降到三票以下，退出到两轮同速。 */
        terrain(15, 0); samples(2, 0, 0); expect(400, 300);
        tick(0, 0); expect(400, 400);
        /* 反向同样在三票时触发，正反切换共用五次窗口。 */
        terrain(15, -0.008); samples(2, 0, 0); expect(400, 400);
        tick(0, 0); expect(300, 400);
        samples(5, 0, 0);
        terrain(15, 0.008); samples(2, 0, 0); expect(300, 400);
        tick(0, 0); expect(400, 300);
        /* 新门槛以下的偏差不触发；下坡按相反轮速方向纠偏。 */
        terrain(15, 0.005); samples(5, 0, 0); expect(400, 400);
        terrain(-15, 0); samples(8, 0, 0); samples(5, 0, 0);
        terrain(-15, 0.008); samples(3, 0, 0); expect(150, 200);
        terrain(-15, 0); samples(3, 0, 0); expect(200, 200);
    }
    else if (strcmp(name, "invalid_y_input") == 0) {
        start(0); climb();
        terrain(15, 0.02); samples(5, 0, 0); expect(400, 300);
        acceleration.x = HUGE_VAL; acceleration.y = 1;
        samples(5, 0, 0); expect(400, 400);
        assert(!kobukiDebug.accelValid && kobukiDebug.yCorrection == 0);
        assert(kobukiDebug.invalidAccelSamples == 5 && kobukiDebug.yPositiveVotes == 0);
        acceleration.x = 0; acceleration.y = 0; acceleration.z = 0;
        samples(8, 0, 0);
        assert(!kobukiDebug.accelValid && kobukiDebug.phase == CLIMB_RAMP);
        terrain(15, -0.02); samples(5, 0, 0); expect(300, 400);
        assert(kobukiDebug.invalidAccelSamples == 0);
    }
    else if (strcmp(name, "finish_distance_wrap") == 0) {
        start(0); descend();
        terrain(0, 0); samples(8, 2147483547, 0);
        tick(2147483647, 0); expect(400, 400);
        assert(kobukiDebug.finishDistance == 100);
        tick((-2147483647 - 1), 0); expect(400, 400);
        assert(kobukiDebug.finishDistance == 101);
        tick(-2147483550, 0); expect(400, 400);
        assert(kobukiDebug.finishDistance == 199);
        tick(-2147483549, 0); expect(0, 0);
    }
    else if (strcmp(name, "finish_distance_frozen") == 0) {
        start(0); finishStraight();
        for (i = 0; i < 1000; i++) { tick(0, 0); }
        assert(kobukiDebug.phase == FINISH_STRAIGHT && kobukiDebug.finishDistance == 0);
        assert(kobukiDebug.stillDistanceSamples >= 1000);
        tick(200, 0); expect(0, 0);
        assert(kobukiDebug.stillDistanceSamples == 0);
    }
    else if (strcmp(name, "ramp_clear_vote_threshold") == 0) {
        start(0);
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        terrain(15, 0); samples(6, -120, -80);
        terrain(0, 0); tick(-120, -80);
        terrain(15, 0); tick(-120, -80);
        assert(kobukiDebug.phase == CLIMB_RAMP && kobukiDebug.motion == ROUTE_DRIVE);
    }
    else if (strcmp(name, "no_slope_during_turn") == 0) {
        start(0);
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        terrain(15, 0.02); samples(10, -119, 0);
        assert(kobukiDebug.phase == APPROACH_RAMP && kobukiDebug.motion == OBSTACLE_BACKUP);
        tick(-120, 0); samples(10, -120, -40);
        assert(kobukiDebug.phase == APPROACH_RAMP && kobukiDebug.motion == OBSTACLE_TURN);
        assert(kobukiDebug.pitchCount == 0 && kobukiDebug.yCorrection == 0);
    }
    else if (strcmp(name, "bad_accel_configuration") == 0) {
        start(0); terrain(15, 0.02); samples(20, 0, 0);
        assert(!kobukiDebug.accelValid && kobukiDebug.phase == APPROACH_RAMP);
    }
    else { return 0; }
    return 1;
}

int main(int argc, char **argv)
{
    int i;
    assert(argc >= 2);
    if (argc >= 3) { simulator = strcmp(argv[2], "simulator") == 0; }
    if (repairScenario(argv[1])) {
        printf("PASS %s (%s)\n", argv[1], simulator ? "simulator" : "hardware interface");
        return 0;
    }
    if (strcmp(argv[1], "persistent_bumper") != 0
        && strcmp(argv[1], "overshoot") != 0
        && strcmp(argv[1], "ramp_during_clear") != 0
        && strcmp(argv[1], "approach_heading_target") != 0
        && strcmp(argv[1], "clear_heading_hold") != 0
        && strcmp(argv[1], "clear_distance_direction") != 0
        && strcmp(argv[1], "repeated_collision_heading") != 0) {
        return combinedRegressionMain(argc, argv);
    }
    start(0);
    if (strcmp(argv[1], "persistent_bumper") == 0) {
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); tick(-120, 0); expect(220, -220);
        /* 参考文件先进入绕行，下一次检测到持续碰撞才再次倒车。 */
        tick(-120, -80); expect(400, 400);
        tick(-120, -80); expect(-240, -240);
        tick(-239, -80); expect(-240, -240);
    }
    else if (strcmp(argv[1], "overshoot") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80); tick(180, -80);
        tick(180, -6); expect(-40, 40);
        /* 回正接近目标时减速，越过目标后应反向，避免继续转整圈。 */
        tick(180, 6); expect(40, -40);
        tick(180, 0); expect(400, 400);
    }
    else if (strcmp(argv[1], "ramp_during_clear") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        terrain(15, 0); samples(8, -120, -80); expect(400, 400);
        /* 绕行确认上坡后立即接管，碰撞不再触发倒车。 */
        input.bumps_wheelDrops.bumpLeft = true;
        tick(-120, -80); expect(400, 400);
        input.bumps_wheelDrops.bumpLeft = false;
        terrain(15, 0.02); samples(5, -100, -80); expect(400, 300);
        terrain(-15, 0); samples(8, 60, -80); expect(200, 200);
        terrain(0, 0); samples(8, 60, -80); expect(400, 400);
        tick(259, -80); expect(400, 400);
        tick(260, -80); expect(0, 0);
    }
    else if (strcmp(argv[1], "approach_heading_target") == 0) {
        start(30);
        tick(0, 40); expect(400, 394);
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 40); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 40); tick(-120, -40); tick(180, -40); expect(-220, 220);
        /* 回到启动航向 30 度，不回到碰撞前偏离的 40 度。 */
        tick(180, 30); expect(400, 400);
    }
    else if (strcmp(argv[1], "clear_heading_hold") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80); expect(400, 400);
        tick(-119, -70); expect(400, 394);
        for (i = 0; i < 20; i++) { tick(-119, -70); }
        assert(leftSpeed == 400 && rightSpeed < 400 && rightSpeed >= 320);
        pauseAt(-119, -70);
        resumeAt(-119, -70); expect(400, 394);
    }
    else if (strcmp(argv[1], "clear_distance_direction") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        tick(-419, -80); expect(400, 400);
        tick(-420, -80); expect(-220, 220);
        tick(-420, 0); expect(400, 400);
    }
    else {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        input.bumps_wheelDrops.bumpLeft = true; tick(-120, -80); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-240, -80); tick(-240, -160); tick(60, -160); expect(-220, 220);
        tick(60, 0); expect(400, 400);
    }
    printf("PASS %s (%s)\n", argv[1], simulator ? "simulator" : "hardware interface");
    return 0;
}
