/* 独立编译两个控制器，通过公开接口对比；只做离线验证。 */
#include "KobukiNavigationStateChart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void ReferenceController(const int16_t, const int32_t, const int32_t,
    const KobukiSensors_t, const accelerometer_t, int16_t *, int16_t *, const bool);
void DebugController(const int16_t, const int32_t, const int32_t,
    const KobukiSensors_t, const accelerometer_t, int16_t *, int16_t *, const bool);

static KobukiSensors_t sensors;
static accelerometer_t acceleration;
static int16_t referenceLeft, referenceRight, debugLeft, debugRight;
static int16_t maximum = 400;
static bool simulator;
static int comparisons;
static const double radiansToDegrees = 57.29577951308232;

static void terrain(const double pitch, const double y, const double scale)
{
    double x = sin(pitch / radiansToDegrees);
    acceleration.x = x * scale;
    acceleration.y = y * scale;
    acceleration.z = sqrt(1.0 - x * x - y * y) * scale;
}

static void tick(const int32_t distance, const int32_t angle)
{
    ReferenceController(maximum, distance, angle, sensors, acceleration,
        &referenceRight, &referenceLeft, simulator);
    DebugController(maximum, distance, angle, sensors, acceleration,
        &debugRight, &debugLeft, simulator);
}

static void expectDebug(const int16_t left, const int16_t right)
{
    if (debugLeft != left || debugRight != right) {
        fprintf(stderr, "debug expected %d/%d, got %d/%d\n", left, right, debugLeft, debugRight);
        assert(0);
    }
}

static void equal(void)
{
    if (referenceLeft != debugLeft || referenceRight != debugRight) {
        fprintf(stderr, "reference %d/%d, debug %d/%d\n",
            referenceLeft, referenceRight, debugLeft, debugRight);
        assert(0);
    }
    comparisons++;
}

static void samples(const int count, const int32_t distance, const int32_t angle)
{
    int i;
    for (i = 0; i < count; i++) { tick(distance, angle); }
}

static void start(void)
{
    memset(&sensors, 0, sizeof(sensors)); terrain(0, 0, 1);
    sensors.buttons.B1 = true; tick(0, 0); equal(); expectDebug(0, 0);
    sensors.buttons.B1 = false; tick(0, 0); equal(); expectDebug(0, 0);
    sensors.buttons.B0 = true; tick(0, 0); equal(); expectDebug(0, 0);
    sensors.buttons.B0 = false; tick(0, 0); equal();
    expectDebug(maximum < 150 ? maximum : 150, maximum < 150 ? maximum : 150);
}

static void climb(const double scale)
{
    int i;
    terrain(15, 0, scale);
    for (i = 0; i < 8; i++) { tick(0, 0); equal(); }
    expectDebug(maximum < 200 ? maximum : 200, maximum < 200 ? maximum : 200);
}

static void descend(const double scale)
{
    terrain(-15, 0, scale); samples(8, 0, 0);
    equal(); expectDebug(maximum < 100 ? maximum : 100, maximum < 100 ? maximum : 100);
}

int main(int argc, char **argv)
{
    int i;
    assert(argc >= 2);
    simulator = argc >= 3 && strcmp(argv[2], "simulator") == 0;
    if (strcmp(argv[1], "speed_cap") == 0) { maximum = 80; }
    start();
    if (strcmp(argv[1], "route_y") == 0 || strcmp(argv[1], "scaled_units") == 0
        || strcmp(argv[1], "speed_cap") == 0) {
        double scale = strcmp(argv[1], "scaled_units") == 0 ? 9.80665 : 1.0;
        climb(scale);
        terrain(15, 0.02, scale);
        for (i = 0; i < 12; i++) { tick(0, 0); equal(); }
        expectDebug(maximum < 200 ? maximum : 200, maximum < 150 ? maximum : 150);
        terrain(15, -0.02, scale);
        for (i = 0; i < 12; i++) { tick(0, 0); equal(); }
        terrain(15, 0, scale); samples(5, 0, 0); equal();
        terrain(0, 0, scale); samples(8, 0, 0); equal();
        descend(scale);
        terrain(-15, 0.02, scale);
        for (i = 0; i < 12; i++) { tick(0, 0); equal(); }
        expectDebug(maximum < 75 ? maximum : 75, maximum < 100 ? maximum : 100);
        terrain(-15, -0.02, scale);
        for (i = 0; i < 12; i++) { tick(0, 0); equal(); }
        terrain(-15, 0, scale); samples(5, 0, 0); equal();
        terrain(0, 0, scale); samples(8, 0, 0);
        expectDebug(0, 0); assert(referenceLeft > 0 && referenceRight > 0);
    }
    else if (strcmp(argv[1], "no_stop_before_descent") == 0) {
        for (i = 0; i < 30; i++) { tick(i * 10, 0); equal(); expectDebug(150, 150); }
        climb(1); terrain(0, 0, 1);
        for (i = 0; i < 30; i++) { tick(300 + i * 10, 0); equal(); }
        expectDebug(150, 150);
    }
    else if (strcmp(argv[1], "downhill_flag_and_stop") == 0) {
        /* 无需先识别上坡；独立 flag 的6/8与参考路线的7/8是有意保留的差异。 */
        samples(8, 0, 0);
        terrain(-15, 0, 9.80665); samples(6, 0, 0); expectDebug(100, 100);
        terrain(0, 0, 9.80665); samples(7, 100, 0); expectDebug(0, 0);
        sensors.buttons.B0 = true;
        sensors.cliffLeft = sensors.bumps_wheelDrops.bumpLeft = true;
        terrain(15, 0.1, 1); samples(12, 200, 90); expectDebug(0, 0);
        start(); expectDebug(150, 150);
    }
    else if (strcmp(argv[1], "landing_boundary") == 0) {
        climb(1); descend(1);
        terrain(4.9, 0, 1); samples(12, 100, 0); expectDebug(100, 100);
        terrain(4.7, 0, 1); samples(6, 200, 0); expectDebug(0, 0);
    }
    else if (strcmp(argv[1], "cliff_parity") == 0) {
        climb(1); terrain(15, 0.02, 1); samples(5, 0, 0); equal();
        sensors.cliffLeft = true; tick(0, 0); equal(); expectDebug(-50, -80);
        tick(-59, 0); equal(); expectDebug(-50, -80);
        tick(-60, 0); equal(); expectDebug(80, -80);
        tick(-60, -30); equal(); expectDebug(200, 150);
        tick(-60, -30); equal(); expectDebug(-50, -80);
        sensors.cliffLeft = false; tick(-120, -30); equal();
        tick(-120, -60); equal(); expectDebug(200, 150);
    }
    else if (strcmp(argv[1], "pause_y") == 0) {
        climb(1); sensors.buttons.B0 = true; tick(0, 0); equal();
        terrain(15, -0.02, 1);
        for (i = 0; i < 5; i++) { tick(0, 0); equal(); }
        sensors.buttons.B0 = false; tick(0, 0); equal();
        sensors.buttons.B0 = true; tick(0, 0); equal();
        sensors.buttons.B0 = false; tick(0, 0); equal(); expectDebug(150, 200);
    }
    else if (strcmp(argv[1], "bumper_extension") == 0) {
        sensors.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0); expectDebug(-168, -168);
        sensors.bumps_wheelDrops.bumpLeft = false;
        tick(-119, 0); expectDebug(-168, -168);
        tick(-120, 0); expectDebug(154, -154);
        tick(-120, -80); expectDebug(150, 150);
    }
    else if (strcmp(argv[1], "flag_survives_pause_cliff") == 0) {
        samples(8, 0, 0);
        terrain(-15, 0, 1); samples(5, 0, 0); expectDebug(150, 150);
        sensors.buttons.B0 = true; tick(0, 0);
        sensors.buttons.B0 = false; tick(0, 0);
        sensors.buttons.B0 = true; tick(0, 0);
        sensors.buttons.B0 = false; tick(0, 0); expectDebug(100, 100);
        sensors.cliffLeft = true; tick(0, 0); expectDebug(-50, -80);
        terrain(0, 0, 1); samples(12, 0, 0); expectDebug(-50, -80);
        tick(-60, 0); expectDebug(80, -80);
        sensors.cliffLeft = false; tick(-60, -30); expectDebug(100, 100);
        samples(7, -60, -30); expectDebug(0, 0);
    }
    else { fprintf(stderr, "Unknown scenario: %s\n", argv[1]); return 2; }
    printf("PASS %s (%s, offline), %d reference comparisons\n",
        argv[1], simulator ? "simulator" : "hardware interface", comparisons);
    return 0;
}
