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

/* 默认接线下额外施加未知安装偏角，验证自动地面参考而非手工配置补偿。 */
static void biasedTerrain(const double pitchDeg, const double pitchBias, const double rollBias)
{
    double x = sin(pitchDeg / RAD_TO_DEG), z = cos(pitchDeg / RAD_TO_DEG);
    double p = pitchBias / RAD_TO_DEG, r = rollBias / RAD_TO_DEG;
    double intermediateZ = -x * sin(p) + z * cos(p);
    acceleration.x = x * cos(p) + z * sin(p);
    acceleration.y = intermediateZ * sin(r);
    acceleration.z = intermediateZ * cos(r);
}

static void startBiased(const double pitchBias, const double rollBias)
{
    memset(&input, 0, sizeof(input));
    biasedTerrain(0, pitchBias, rollBias);
    input.buttons.B1 = true; tick(0, 0); expect(0, 0);
    input.buttons.B1 = false; samples(FLOOR_REFERENCE_SAMPLE_COUNT, 0, 0); expect(0, 0);
    assert(kobukiDebug.floorReferenceValid);
    input.buttons.B0 = true; tick(0, 0); expect(0, 0);
    input.buttons.B0 = false; tick(0, 0); expect(400, 400);
}

static void reachTop(void)
{
    start(0); climb(); terrain(0, 0); samples(PITCH_WINDOW_SAMPLE_COUNT, 0, 0);
    assert(kobukiDebug.finishStage == FINISH_WAIT_DOWNHILL);
    assert(!kobukiDebug.descentConfirmed);
}

static int crestStopScenario(const char * const name)
{
    int i, j;
    int32_t distance = 0;
    if (strcmp(name, "crest_braking_before_top") == 0) {
        start(0); climb();
        /* 平面 Z=1g，只有减速造成的 X=-0.07g；没有真正的坡面重力倾斜。 */
        acceleration.x = -0.07; acceleration.y = 0; acceleration.z = 1;
        for (i = 0; i < 12; i++) { tick((i + 1) * 20, 0); }
        assert(kobukiDebug.finishStage == FINISH_WAIT_TOP && !kobukiDebug.descentConfirmed);
        terrain(0, 0); samples(16, 300, 0); tick(1000, 0); expect(300, 300);
        assert(!kobukiDebug.finished && kobukiDebug.finishDistance == 0);
    }
    else if (strcmp(name, "crest_braking_after_top") == 0) {
        reachTop(); acceleration.x = -0.07; acceleration.y = 0; acceleration.z = 1;
        for (i = 1; i <= 5; i++) { tick(i * 20, 0); }
        assert(!kobukiDebug.descentConfirmed && kobukiDebug.descentDistance == 80);
        terrain(0, 0); samples(8, 120, 0); tick(1000, 0); expect(300, 300);
        assert(kobukiDebug.descentDistance == 0 && kobukiDebug.finishDistance == 0);
        /* 即便减速读数超过主路线的负坡度门槛，距离不足也不能切到下坡。 */
        acceleration.x = -0.12; acceleration.y = 0; acceleration.z = 1;
        for (i = 1; i <= 8; i++) { tick(1000 + i * 10, 0); }
        assert(!kobukiDebug.descentConfirmed && kobukiDebug.phase == DRIVE_ACROSS_TOP);
        terrain(0, 0); samples(8, 1100, 0); expect(300, 300);
        confirmDescentAt(2000, 0);
        terrain(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 2000, 0);
        tick(2199, 0); expect(400, 400); tick(2200, 0); expect(0, 0);
    }
    else if (strcmp(name, "descent_distance_threshold") == 0) {
        reachTop(); terrain(-4, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 0, 0);
        tick(FINISH_DESCENT_MIN_DISTANCE_MM - 1, 0);
        assert(!kobukiDebug.descentConfirmed);
        tick(FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        assert(kobukiDebug.descentConfirmed);
        terrain(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT, FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        tick(FINISH_DESCENT_MIN_DISTANCE_MM + 199, 0); expect(400, 400);
        tick(FINISH_DESCENT_MIN_DISTANCE_MM + 200, 0); expect(0, 0);
    }
    else if (strcmp(name, "crest_repeated_pulses") == 0) {
        reachTop();
        for (i = 0; i < 20; i++) {
            acceleration.x = -0.07; acceleration.y = 0; acceleration.z = 1;
            for (j = 0; j < 5; j++) { distance += 10; tick(distance, 0); }
            terrain(0, 0); distance += 10; samples(5, distance, 0);
            assert(!kobukiDebug.descentConfirmed && kobukiDebug.descentDistance == 0);
        }
        expect(300, 300); assert(!kobukiDebug.finished);
    }
    else if (strcmp(name, "descent_pause_resets_candidate") == 0) {
        reachTop(); terrain(-4, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 0, 0); tick(100, 0);
        assert(kobukiDebug.descentDistance == 100);
        pauseAt(100, 0); tick(5000, 0); resumeAt(5000, 0);
        assert(kobukiDebug.descentDistance == 0);
        tick(5000 + FINISH_DESCENT_MIN_DISTANCE_MM - 1, 0);
        samples(FINISH_VOTE_SAMPLE_COUNT, 5000 + FINISH_DESCENT_MIN_DISTANCE_MM - 1, 0);
        assert(!kobukiDebug.descentConfirmed);
        tick(5000 + FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        assert(kobukiDebug.descentConfirmed);
    }
    else if (strcmp(name, "descent_invalid_gap") == 0) {
        reachTop(); terrain(-4, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 0, 0); tick(100, 0);
        acceleration.x = acceleration.y = acceleration.z = 0;
        samples(FINISH_VOTE_SAMPLE_COUNT, 10000, 0);
        assert(kobukiDebug.descentDistance == 0);
        terrain(-4, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 10000, 0);
        assert(!kobukiDebug.descentConfirmed && kobukiDebug.descentDistance == 0);
        tick(10000 + FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        assert(kobukiDebug.descentConfirmed);
    }
    else if (strcmp(name, "descent_distance_wrap") == 0) {
        reachTop(); terrain(-4, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 2147483547, 0);
        tick(2147483647, 0); assert(!kobukiDebug.descentConfirmed);
        tick(-2147483599, 0); assert(kobukiDebug.descentConfirmed);
        terrain(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT, -2147483599, 0);
        tick(-2147483400, 0); expect(400, 400); tick(-2147483399, 0); expect(0, 0);
    }
    else { return 0; }
    return 1;
}

static int redesignedStopScenario(const char * const name)
{
    int i;
    if (strcmp(name, "stop_floor_bias") == 0) {
        startBiased(4, 3);
        assert(fabs(kobukiDebug.pitchDeg) < 0.001 && kobukiDebug.tiltDeg < 0.001);
        biasedTerrain(15, 4, 3); samples(8, 0, 0);
        assert(kobukiDebug.phase == CLIMB_RAMP);
        biasedTerrain(0, 4, 3); samples(24, 100, 0); expect(300, 300);
        assert(!kobukiDebug.descentConfirmed && kobukiDebug.finishDistance == 0);
        biasedTerrain(-15, 4, 3); samples(8, 200, 0);
        tick(200 + FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        assert(kobukiDebug.descentConfirmed);
        biasedTerrain(0, 4, 3); samples(8, 300, 0); expect(400, 400);
        tick(499, 0); expect(400, 400); tick(500, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_shallow_descent") == 0) {
        start(0); climb(); terrain(0, 0); samples(8, 0, 0);
        terrain(-4, 0); samples(16, 100, 0); expect(300, 300);
        tick(100 + FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        expect(200, 200);
        assert(kobukiDebug.descentConfirmed && kobukiDebug.finishStage == FINISH_WAIT_GROUND);
        terrain(0, 0); samples(8, 300, 0); expect(400, 400);
        tick(499, 0); expect(400, 400); tick(500, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_noisy_level") == 0) {
        start(0); descend();
        terrain(4, 0); samples(FINISH_VOTE_SAMPLE_COUNT - FINISH_LEVEL_REQUIRED, 0, 0);
        terrain(0, 0); samples(FINISH_LEVEL_REQUIRED - 1, 0, 0); expect(200, 200);
        tick(0, 0); expect(400, 400);
        assert(kobukiDebug.finishStage == FINISH_COUNT_DISTANCE);
        tick(199, 0); expect(400, 400); tick(200, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_reject_five_level_votes") == 0) {
        start(0); descend();
        for (i = 0; i < 80; i++) {
            terrain(i % FINISH_VOTE_SAMPLE_COUNT < FINISH_LEVEL_REQUIRED - 1 ? 0 : 4, 0); tick(i * 20, 0);
        }
        assert(kobukiDebug.finishStage == FINISH_WAIT_GROUND && kobukiDebug.finishDistance == 0);
        expect(200, 200);
    }
    else if (strcmp(name, "stop_reject_top") == 0) {
        start(0); terrain(0, 0); samples(32, 100, 0);
        assert(kobukiDebug.finishStage == FINISH_WAIT_UPHILL);
        climb(); terrain(0, 0); samples(32, 1000, 0); expect(300, 300);
        for (i = 0; i < 80; i++) {
            terrain(i % 8 < 2 ? -4 : 0, 0); tick(1000 + i * 20, 0);
        }
        assert(!kobukiDebug.descentConfirmed && kobukiDebug.finishDistance == 0);
        expect(300, 300);
    }
    else if (strcmp(name, "stop_pause_keeps_descent") == 0) {
        start(0); descend(); terrain(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT - 1, 0, 0);
        pauseAt(0, 0); assert(kobukiDebug.descentConfirmed);
        resumeAt(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT - 2, 0, 0); expect(200, 200);
        assert(kobukiDebug.finishSampleCount == FINISH_VOTE_SAMPLE_COUNT - 1);
        tick(0, 0); expect(400, 400);
        tick(200, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_reference_fixed_on_resume") == 0) {
        start(0); climb(); pauseAt(0, 0); samples(32, 0, 0); resumeAt(0, 0);
        assert(fabs(kobukiDebug.floorX) < 0.001 && fabs(kobukiDebug.pitchDeg - 15) < 0.001);
        terrain(0, 0); samples(PITCH_WINDOW_SAMPLE_COUNT, 0, 0);
        confirmDescentAt(0, 0);
        terrain(0, 0); samples(8, 0, 0); tick(200, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_start_reference_guard") == 0) {
        memset(&input, 0, sizeof(input));
        acceleration.x = acceleration.y = acceleration.z = 0;
        input.buttons.B1 = true; tick(0, 0);
        input.buttons.B1 = false; input.buttons.B0 = true; tick(0, 0);
        input.buttons.B0 = false; samples(16, 0, 0); expect(0, 0);
        assert(!kobukiDebug.floorReferenceValid);
        for (i = 0; i < 16; i++) { terrain(i % 2 ? 4 : -4, 0); tick(0, 0); expect(0, 0); }
        assert(!kobukiDebug.floorReferenceValid);
        terrain(0, 0); samples(8, 0, 0); expect(400, 400);
        assert(kobukiDebug.floorReferenceValid);
        input.buttons.B1 = true; tick(0, 0); expect(0, 0);
        assert(!kobukiDebug.floorReferenceValid && !kobukiDebug.descentConfirmed);
        input.buttons.B1 = false; samples(16, 0, 0); expect(0, 0);
        /* 启动前搬动后的新姿态要重采，不能使用上电时的旧参考。 */
        biasedTerrain(0, 4, 0); samples(FLOOR_REFERENCE_SAMPLE_COUNT - 1, 0, 0); expect(0, 0);
        assert(!kobukiDebug.floorReferenceValid);
        tick(0, 0); assert(kobukiDebug.floorReferenceValid);
        /* 参考已采好，但当前输入无效时也不能启动。 */
        acceleration.x = acceleration.y = acceleration.z = 0;
        input.buttons.B0 = true; tick(0, 0);
        input.buttons.B0 = false; samples(8, 0, 0); expect(0, 0);
        terrain(0, 0); samples(FLOOR_REFERENCE_SAMPLE_COUNT - 1, 0, 0); expect(0, 0);
        tick(0, 0); expect(400, 400);
    }
    else if (strcmp(name, "stop_invalid_level") == 0) {
        start(0); descend(); terrain(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT - 1, 0, 0);
        acceleration.x = acceleration.y = acceleration.z = 0;
        samples(16, 1000, 0); expect(200, 200);
        assert(kobukiDebug.finishStage == FINISH_WAIT_GROUND);
        terrain(0, 0); samples(8, 1000, 0);
        tick(1199, 0); expect(400, 400); tick(1200, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_vote_outlier_last") == 0) {
        start(0); pauseAt(0, 0);
        terrain(15, 0); resumeAt(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT - 2, 0, 0);
        assert(kobukiDebug.finishStage == FINISH_WAIT_UPHILL);
        acceleration.x = acceleration.y = acceleration.z = 0;
        tick(0, 0);
        assert(kobukiDebug.finishStage == FINISH_WAIT_TOP);
        terrain(0, 0); samples(FINISH_VOTE_SAMPLE_COUNT, 0, 0);
        terrain(-4, 0); samples(FINISH_VOTE_SAMPLE_COUNT - 1, 0, 0);
        acceleration.x = acceleration.y = acceleration.z = 0;
        tick(0, 0);
        assert(!kobukiDebug.descentConfirmed);
        terrain(-4, 0); tick(0, 0); tick(FINISH_DESCENT_MIN_DISTANCE_MM, 0);
        assert(kobukiDebug.descentConfirmed);
        terrain(0, 0); samples(8, 0, 0);
        tick(200, 0); expect(0, 0);
    }
    else if (strcmp(name, "stop_latest_level_required") == 0) {
        start(0); descend(); terrain(0, 0); samples(FINISH_LEVEL_REQUIRED, 0, 0);
        terrain(4, 0); samples(FINISH_VOTE_SAMPLE_COUNT - FINISH_LEVEL_REQUIRED, 0, 0); expect(200, 200);
        assert(kobukiDebug.finishVotes == FINISH_LEVEL_REQUIRED && kobukiDebug.finishSampleCount == FINISH_VOTE_SAMPLE_COUNT);
        terrain(0, 0); tick(0, 0); expect(400, 400);
        tick(200, 0); expect(0, 0);
    }
    else { return 0; }
    return 1;
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
        tick(FINISH_DESCENT_MIN_DISTANCE_MM * DISTANCE_FORWARD_SIGN, 0);
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
        configuredTerrain(0, 0); samples(8, 0, 0);
        configuredTerrain(-15, 0); samples(8, 0, 0);
        tick(FINISH_DESCENT_MIN_DISTANCE_MM * DISTANCE_FORWARD_SIGN, 0);
        configuredTerrain(0, 0); samples(8, 0, 0);
        tick(50 * DISTANCE_FORWARD_SIGN, 0);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(50 * DISTANCE_FORWARD_SIGN, 0); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-70 * DISTANCE_FORWARD_SIGN, 0); expect(220, -220);
        tick(-70 * DISTANCE_FORWARD_SIGN, -80); expect(400, 400);
        tick((OBSTACLE_CLEAR_DISTANCE_MM - 70) * DISTANCE_FORWARD_SIGN, -80); expect(-220, 220);
        tick((OBSTACLE_CLEAR_DISTANCE_MM - 70) * DISTANCE_FORWARD_SIGN, 0); expect(400, 400);
        assert(kobukiDebug.finishDistance == 50);
        tick((OBSTACLE_CLEAR_DISTANCE_MM + 79) * DISTANCE_FORWARD_SIGN, 0); expect(400, 400);
        tick((OBSTACLE_CLEAR_DISTANCE_MM + 80) * DISTANCE_FORWARD_SIGN, 0); expect(0, 0);
    }
    else if (strcmp(name, "y_limited_pair") == 0) {
        maximum = 200; start(0); climb();
        terrain(15, 0.02); samples(5, 0, 0); expect(200, 150);
        terrain(15, -0.02); samples(5, 0, 0); expect(150, 200);
        maximum = 100;
        terrain(0, 0); samples(8, 0, 0); confirmDescentAt(0, 0);
        terrain(-15, 0.02); samples(5, 0, 0); expect(75, 100);
        terrain(-15, -0.02); samples(5, 0, 0); expect(100, 75);
        assert(kobukiDebug.yCorrection == -1 && kobukiDebug.maxWheelSpeed == 100);
    }
    else if (strcmp(name, "y_sensitive_switching") == 0) {
        start(0); climb(); samples(Y_FILTER_SAMPLE_COUNT, 0, 0);
        /* 触发/退出与反向均遵守当前参数，不绑定某次调参的固定票数。 */
        terrain(15, Y_ALIGNMENT_THRESHOLD_G * 2); samples(Y_REQUIRED_SAMPLE_COUNT - 1, 0, 0); expect(400, 400);
        tick(0, 0); expect(400, 300);
        terrain(15, 0); samples(Y_FILTER_SAMPLE_COUNT, 0, 0); expect(400, 400);
        terrain(15, -Y_ALIGNMENT_THRESHOLD_G * 2); samples(Y_REQUIRED_SAMPLE_COUNT - 1, 0, 0); expect(400, 400);
        tick(0, 0); expect(300, 400);
        terrain(15, Y_ALIGNMENT_THRESHOLD_G * 2); samples(Y_FILTER_SAMPLE_COUNT, 0, 0); expect(400, 300);
        terrain(15, Y_ALIGNMENT_THRESHOLD_G / 2); samples(Y_FILTER_SAMPLE_COUNT, 0, 0); expect(400, 400);
        terrain(0, 0); samples(8, 0, 0); confirmDescentAt(0, 0);
        terrain(-15, Y_ALIGNMENT_THRESHOLD_G * 2); samples(Y_FILTER_SAMPLE_COUNT, 0, 0); expect(150, 200);
        terrain(-15, 0); samples(Y_FILTER_SAMPLE_COUNT, 0, 0); expect(200, 200);
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
        memset(&input, 0, sizeof(input)); terrain(0, 0);
        input.buttons.B1 = true; tick(0, 0);
        input.buttons.B1 = false; input.buttons.B0 = true; tick(0, 0);
        input.buttons.B0 = false;
        terrain(15, 0.02); samples(20, 0, 0); expect(0, 0);
        assert(!kobukiDebug.accelValid && kobukiDebug.phase == APPROACH_RAMP);
        assert(!kobukiDebug.floorReferenceValid);
    }
    else { return 0; }
    return 1;
}

int main(int argc, char **argv)
{
    int i;
    assert(argc >= 2);
    if (argc >= 3) { simulator = strcmp(argv[2], "simulator") == 0; }
    if (crestStopScenario(argv[1]) || redesignedStopScenario(argv[1]) || repairScenario(argv[1])) {
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
        tick(-120, 0); tick(-120, -80); tick(TEST_CLEAR_END, -80);
        tick(TEST_CLEAR_END, -6); expect(-40, 40);
        /* 回正接近目标时减速，越过目标后应反向，避免继续转整圈。 */
        tick(TEST_CLEAR_END, 6); expect(40, -40);
        tick(TEST_CLEAR_END, 0); expect(400, 400);
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
        terrain(0, 0); samples(8, -100, -80); confirmDescentAt(60, -80);
        terrain(0, 0); samples(8, 60, -80); expect(400, 400);
        tick(259, -80); expect(400, 400);
        tick(260, -80); expect(0, 0);
    }
    else if (strcmp(argv[1], "approach_heading_target") == 0) {
        start(30);
        tick(0, 40); expect(400, 394);
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 40); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 40); tick(-120, -40); tick(TEST_CLEAR_END, -40); expect(-220, 220);
        /* 回到启动航向 30 度，不回到碰撞前偏离的 40 度。 */
        tick(TEST_CLEAR_END, 30); expect(400, 400);
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
        tick(-120 - OBSTACLE_CLEAR_DISTANCE_MM + 1, -80); expect(400, 400);
        tick(-120 - OBSTACLE_CLEAR_DISTANCE_MM, -80); expect(-220, 220);
        tick(-120 - OBSTACLE_CLEAR_DISTANCE_MM, 0); expect(400, 400);
    }
    else {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        input.bumps_wheelDrops.bumpLeft = true; tick(-120, -80); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-240, -80); tick(-240, -160); tick(OBSTACLE_CLEAR_DISTANCE_MM - 240, -160); expect(-220, 220);
        tick(OBSTACLE_CLEAR_DISTANCE_MM - 240, 0); expect(400, 400);
    }
    printf("PASS %s (%s)\n", argv[1], simulator ? "simulator" : "hardware interface");
    return 0;
}
