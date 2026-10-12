/* Each scenario runs in a fresh process because the controller retains state. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../KobukiNavigationStateChart_obstacle.c"

static KobukiSensors_t input;
static accelerometer_t acceleration;
static int16_t leftSpeed;
static int16_t rightSpeed;
static int16_t speedLimit = 200;

static void tick(const int32_t distance, const int32_t angle)
{
    KobukiNavigationStatechart(speedLimit, distance, angle, input,
        acceleration, &rightSpeed, &leftSpeed, false);
    assert(abs(leftSpeed) <= speedLimit);
    assert(abs(rightSpeed) <= speedLimit);
}

static void expect(const int16_t left, const int16_t right)
{
    assert(leftSpeed == left);
    assert(rightSpeed == right);
}

static void start(const int32_t angle)
{
    tick(0, angle);
    expect(0, 0);
    input.buttons.B0 = true;
    tick(0, angle);
    expect(0, 0);
    input.buttons.B0 = false;
    tick(0, angle);
    expect(limitSpeed(DRIVE_SPEED_MM_S, speedLimit),
        limitSpeed(DRIVE_SPEED_MM_S, speedLimit));
}

static void pause(const int32_t distance, const int32_t angle)
{
    input.buttons.B0 = true;
    tick(distance, angle);
    expect(0, 0);
    input.buttons.B0 = false;
    tick(distance, angle);
    expect(0, 0);
}

static void resume(const int32_t distance, const int32_t angle)
{
    input.buttons.B0 = true;
    tick(distance, angle);
    expect(0, 0);
    input.buttons.B0 = false;
    tick(distance, angle);
}

static void beginRightAvoidance(void)
{
    input.bumps_wheelDrops.bumpLeft = true;
    tick(0, 0);
    expect(-120, -120);
    input.bumps_wheelDrops.bumpLeft = false;
    tick(-119, 0);
    expect(-120, -120);
    tick(-120, 0);
    expect(110, -110);
}

int main(int argc, char **argv)
{
    int leftTurn;
    int32_t turnAngle;

    assert(argc == 2);
    if (strcmp(argv[1], "speed_cap") == 0) {
        speedLimit = 40;
        start(0);
        expect(40, 40);
        input.bumps_wheelDrops.bumpLeft = true;
        tick(0, 0);
        expect(-40, -40);
        input.bumps_wheelDrops.bumpLeft = false;
        tick(-120, 0);
        expect(40, -40);
        tick(-120, -80);
        expect(40, 40);
        tick(180, -80);
        expect(-40, 40);
        tick(180, 6);
        expect(20, -20);
    }
    else if (strcmp(argv[1], "angle_wrap") == 0) {
        start(350);
        input.bumps_wheelDrops.bumpRight = true;
        tick(0, 350);
        input.bumps_wheelDrops.bumpRight = false;
        tick(-120, 350);
        expect(-110, 110);
        tick(-120, 70);
        expect(200, 200);
        tick(180, 70);
        expect(110, -110);
        tick(180, -10);
        expect(200, 200);
    }
    else {
        start(0);
        if (strcmp(argv[1], "nominal_left") == 0
            || strcmp(argv[1], "nominal_right") == 0
            || strcmp(argv[1], "nominal_center") == 0) {
            leftTurn = strcmp(argv[1], "nominal_right") == 0;
            input.bumps_wheelDrops.bumpRight = leftTurn;
            input.bumps_wheelDrops.bumpLeft =
                strcmp(argv[1], "nominal_left") == 0;
            input.bumps_wheelDrops.bumpCenter =
                strcmp(argv[1], "nominal_center") == 0;
            tick(0, 0);
            expect(-120, -120);
            memset(&input, 0, sizeof(input));
            tick(-120, 0);
            expect(leftTurn ? -110 : 110, leftTurn ? 110 : -110);
            turnAngle = leftTurn ? 80 : -80;
            tick(-120, turnAngle);
            expect(200, 200);
            tick(179, turnAngle);
            expect(200, 200);
            tick(180, turnAngle);
            expect(leftTurn ? 110 : -110, leftTurn ? -110 : 110);
            tick(180, 0);
            expect(200, 200);
        }
        else if (strcmp(argv[1], "persistent_bumper") == 0) {
            input.bumps_wheelDrops.bumpLeft = true;
            tick(0, 0);
            tick(-120, 0);
            tick(-120, -80);
            expect(-120, -120);
            tick(-239, -80);
            expect(-120, -120);
            input.bumps_wheelDrops.bumpLeft = false;
            tick(-240, -80);
            expect(110, -110);
            tick(-240, -160);
            expect(200, 200);
        }
        else if (strcmp(argv[1], "resume_bumper") == 0) {
            pause(0, 0);
            input.bumps_wheelDrops.bumpCenter = true;
            resume(0, 0);
            expect(-120, -120);
        }
        else if (strcmp(argv[1], "overshoot") == 0) {
            beginRightAvoidance();
            tick(-120, -80);
            tick(180, -80);
            expect(-110, 110);
            tick(180, -6);
            expect(-20, 20);
            tick(180, 6);
            expect(20, -20);
            tick(180, 0);
            expect(200, 200);
        }
        else if (strcmp(argv[1], "resume_turn") == 0) {
            beginRightAvoidance();
            pause(-120, -40);
            input.bumps_wheelDrops.bumpLeft = true;
            resume(-120, -80);
            expect(-120, -120);
        }
        else if (strcmp(argv[1], "repeat_collision") == 0) {
            beginRightAvoidance();
            tick(-120, -80);
            input.bumps_wheelDrops.bumpRight = true;
            tick(0, -80);
            expect(-120, -120);
            input.bumps_wheelDrops.bumpRight = false;
            tick(-120, -80);
            expect(-110, 110);
            tick(-120, 0);
            expect(200, 200);
            tick(180, 0);
            expect(0, 0);
            tick(180, 0);
            expect(200, 200);
        }
        else {
            assert(!"unknown scenario");
        }
    }
    printf("PASS %s\n", argv[1]);
    return 0;
}
