/* 用真实课程头文件和 LED 写入桩检查状态灯，并复用完整路线回归测试。
 * 本测试不连接 myRIO，不代表实机 LED 已验证。
 */
#define COMBINED_CONTROLLER_SOURCE "../KobukiNavigationStateChart_combined_debug.c"
#define main combinedRegressionMain
#include "test_combined_controller.c"
#undef main

NiFpga_Session myrio_session = 1;
static unsigned int ledWrites = 0;
static uint8_t observedMask = 0;
static NiFpga_Status ledStatus = 0;

NiFpga_Status NiFpga_WriteU8(NiFpga_Session session, uint32_t control, uint8_t value)
{
    assert(session == myrio_session);
    assert(control == DOLED30);
    assert(value <= 0x0F);
    ledWrites++;
    if (ledStatus >= 0) { observedMask = value; }
    return ledStatus;
}

static void expectLed(const uint8_t mask)
{
    assert(observedMask == mask);
}

int main(int argc, char **argv)
{
    unsigned int previous;
    int result;
    if (argc > 1) {
        result = combinedRegressionMain(argc, argv);
        if (simulator) { assert(ledWrites == 0); }
        else { assert(ledWrites > 0); }
        return result;
    }

    /* 正常路线、停车锁止、重置；相同状态不重复写寄存器。 */
    tick(0, 0); expectLed(0x00);
    start(0); expectLed(0x01);
    previous = ledWrites;
    samples(12, 0, 0); expectLed(0x01); assert(ledWrites == previous);
    climb(); expectLed(0x02);
    terrain(0, 0); samples(8, 0, 0); expectLed(0x03);
    terrain(-15, 0); samples(8, 0, 0); expectLed(0x04);
    terrain(0, 0); samples(8, 0, 0); expectLed(0x05);
    tick(200, 0); expect(0, 0); expectLed(0x0F);
    input.buttons.B0 = true; tick(200, 0); expectLed(0x0F);
    start(0); expectLed(0x01);

    /* 避障四个动作，各自显示不同灯码；暂停灭灯，恢复显示当前动作。 */
    input.bumps_wheelDrops.bumpLeft = true;
    tick(0, 0); expectLed(0x06);
    pauseAt(0, 0); expectLed(0x00);
    input.bumps_wheelDrops.bumpLeft = false;
    resumeAt(0, 0); expectLed(0x06);
    tick(-120, 0); expectLed(0x07);
    tick(-120, -80); expectLed(0x08);
    tick(180, -80); expectLed(0x09);
    tick(180, 0); expectLed(0x01);

    /* 悬崖打断避障：灯码切到悬崖动作，完成后恢复路线。 */
    input.bumps_wheelDrops.bumpLeft = true; tick(180, 0); expectLed(0x06);
    input.cliffLeft = true; tick(180, 0); expectLed(0x0A);
    tick(120, 0); expectLed(0x0B);
    input.cliffLeft = input.bumps_wheelDrops.bumpLeft = false;
    tick(120, -30); expectLed(0x01);

    /* 绕障中确认上坡，直接显示爬坡状态，bumper 不覆盖路线灯码。 */
    start(0);
    input.bumps_wheelDrops.bumpLeft = true; tick(0, 0);
    input.bumps_wheelDrops.bumpLeft = false;
    tick(-120, 0); tick(-120, -80); expectLed(0x08);
    terrain(15, 0); samples(8, -120, -80); expectLed(0x02);
    input.bumps_wheelDrops.bumpLeft = true; tick(-120, -80); expectLed(0x02);

    /* 失败时保留旧缓存，下一周期重试；灯控错误不改变暂停/行驶轮速。 */
    ledStatus = -1;
    previous = ledWrites;
    input.buttons.B0 = true; tick(-120, -80); expect(0, 0); expectLed(0x02);
    assert(ledWrites == previous + 1);
    ledStatus = 0;
    tick(-120, -80); expectLed(0x00); assert(ledWrites == previous + 2);
    previous = ledWrites;
    tick(-120, -80); assert(ledWrites == previous);

    /* 仿真模式禁止写硬件；切回实机接口后强制同步一次当前灯码。 */
    simulator = true;
    start(0); climb(); expect(400, 400);
    assert(ledWrites == previous);
    simulator = false;
    tick(0, 0); expectLed(0x02); assert(ledWrites == previous + 1);
    input.buttons.B1 = true; tick(0, 0); expectLed(0x00); expect(0, 0);
    puts("PASS debug LED transitions, pause/reset/latch, write retry and simulator isolation.");
    return 0;
}
