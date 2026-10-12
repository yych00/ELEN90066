/* 验证参考文件的前半段碰撞避障行为，并复用原坡道/终点回归测试。 */
#define COMBINED_CONTROLLER_SOURCE "../KobukiNavigationStateChart_combined_debug.c"
#define main combinedRegressionMain
#include "test_combined_controller.c"
#undef main

int main(int argc, char **argv)
{
    int i;
    assert(argc >= 2);
    if (argc >= 3) { simulator = strcmp(argv[2], "simulator") == 0; }
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
        tick(180, -6); expect(-220, 220);
        /* 保留参考文件的固定方向：过头仍朝原方向转，不使用比例减速。 */
        tick(180, 6); expect(-220, 220);
        tick(180, 0); expect(400, 400);
    }
    else if (strcmp(argv[1], "ramp_during_clear") == 0) {
        input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0); tick(-120, -80);
        terrain(15, 0); samples(8, -120, -80); expect(400, 400);
        /* 绕行期间保留接近坡道阶段及碰撞处理，回正后才确认上坡。 */
        input.bumps_wheelDrops.bumpLeft = true;
        tick(-120, -80); expect(-240, -240);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-240, -80); tick(-240, -160); tick(60, -160); tick(60, 0);
        samples(8, 60, 0); expect(400, 400);
        input.bumps_wheelDrops.bumpLeft = true; tick(60, 0); expect(400, 400);
        terrain(-15, 0); samples(8, 60, 0); expect(200, 200);
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
