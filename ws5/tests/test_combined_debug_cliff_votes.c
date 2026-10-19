/* 离线回归：悬崖动作冻结坡道投票，恢复后继续检测下坡和落地。 */
#define COMBINED_CONTROLLER_SOURCE "../KobukiNavigationStateChart_combined_debug.c"
#define main combinedRegressionMain
#include "test_combined_controller.c"
#undef main

/* 避让时故意送入相反地形，验证这些读数没有混入坡度窗口。 */
static void cliffAt(const int32_t distance, const int32_t angle)
{
    input.cliffLeft = true;
    tick(distance, angle); expect(-100, -160);
    terrain(15, 0); samples(12, distance, angle); expect(-100, -160);
    tick(distance - 60, angle); expect(160, -160);
    terrain(0, 0); samples(12, distance - 60, angle); expect(160, -160);
    input.cliffLeft = false;
    tick(distance - 60, angle - 30);
}

static void top(void)
{
    climb(); terrain(0, 0); samples(8, 0, 0); expect(300, 300);
}

int main(int argc, char **argv)
{
    assert(argc >= 2);
    if (argc >= 3) { simulator = strcmp(argv[2], "simulator") == 0; }
    start(0);
    if (strcmp(argv[1], "cliff_downhill_votes") == 0) {
        top(); terrain(-15, 0); samples(6, 0, 0); expect(300, 300);
        cliffAt(0, 0); expect(300, 300);
        terrain(-15, 0); tick(-60, -30); expect(300, 300);
        tick(-60, -30); expect(200, 200);
        terrain(0, 0); samples(8, -60, -30); expect(400, 400);
        tick(139, -30); expect(400, 400);
        tick(140, -30); expect(0, 0);
    }
    else if (strcmp(argv[1], "cliff_landing_votes") == 0) {
        descend(); terrain(0, 0); samples(6, 0, 0); expect(200, 200);
        cliffAt(0, 0); expect(200, 200);
        terrain(0, 0); tick(-60, -30); expect(200, 200);
        tick(-60, -30); expect(400, 400);
        tick(139, -30); expect(400, 400);
        tick(140, -30); expect(0, 0);
    }
    else if (strcmp(argv[1], "cliff_repeated_votes") == 0) {
        top(); terrain(-15, 0); samples(6, 0, 0);
        input.cliffLeft = true; tick(0, 0); expect(-100, -160);
        tick(-60, 0); expect(160, -160);
        tick(-60, -30); expect(-100, -160);
        samples(12, -60, -30); expect(-100, -160);
        tick(-120, -30); expect(160, -160);
        input.cliffLeft = false; tick(-120, -60); expect(300, 300);
        terrain(-15, 0); samples(2, -120, -60); expect(200, 200);
        terrain(0, 0); samples(6, -120, -60);
        cliffAt(-120, -60); expect(200, 200);
        terrain(0, 0); samples(2, -180, -90); expect(400, 400);
        tick(20, -90); expect(0, 0);
    }
    else if (strcmp(argv[1], "cliff_ground_votes_reset") == 0) {
        terrain(15, 0); samples(6, 0, 0);
        cliffAt(0, 0); expect(400, 400);
        terrain(15, 0); samples(6, -60, -30); expect(400, 400);
        /* 若地面窗口被保留，这里早已进入爬坡，随后会转为坡顶 300。 */
        terrain(0, 0); samples(8, -60, -30); expect(400, 400);
        input.bumps_wheelDrops.bumpCenter = true;
        tick(-60, -30); expect(-240, -240);
    }
    else if (strcmp(argv[1], "cliff_y_window_reset") == 0) {
        climb(); terrain(15, 0.05); samples(5, 0, 0); expect(400, 300);
        cliffAt(0, 0); expect(400, 400);
        terrain(15, 0.05); samples(2, -60, -30); expect(400, 400);
        samples(3, -60, -30); expect(400, 300);
    }
    else if (strcmp(argv[1], "cliff_pause_votes_reset") == 0) {
        top(); terrain(-15, 0); samples(6, 0, 0);
        cliffAt(0, 0); expect(300, 300);
        terrain(-15, 0); pauseAt(-60, -30); resumeAt(-60, -30);
        samples(5, -60, -30); expect(300, 300);
        samples(2, -60, -30); expect(200, 200);
    }
    else if (strcmp(argv[1], "cliff_rolling_votes") == 0) {
        top(); terrain(15, 0); samples(2, 0, 0);
        terrain(-15, 0); samples(6, 0, 0); expect(300, 300);
        cliffAt(0, 0); expect(300, 300);
        /* 保留已满窗口和写入位置，新的一票应替换最老的上坡票。 */
        terrain(-15, 0); tick(-60, -30); expect(200, 200);
    }
    else { return combinedRegressionMain(argc, argv); }
    printf("PASS %s (%s, offline)\n", argv[1], simulator ? "simulator" : "hardware interface");
    return 0;
}
