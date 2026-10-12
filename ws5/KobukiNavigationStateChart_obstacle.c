/*
 * Task 2 - Kobuki obstacle-avoidance state machine.
 *
 * This file is based on the Workshop 4 default square-driving controller.
 * It keeps the supplied pause/resume region and replaces the square path with
 * a non-blocking obstacle-avoidance sequence.
 */

#include "KobukiNavigationStateChart.h"
#include <stdlib.h>

typedef enum {
    INITIAL = 0,
    PAUSE_WAIT_BUTTON_RELEASE,
    UNPAUSE_WAIT_BUTTON_PRESS,
    UNPAUSE_WAIT_BUTTON_RELEASE,

    /* Running-region states. */
    DRIVE,
    BACK_UP,
    TURN_AWAY,
    CLEAR_OBSTACLE,
    REORIENT
} robotState_t;

typedef enum {
    TURN_LEFT = -1,
    TURN_RIGHT = 1
} turnDirection_t;

/* Distances are in mm and angles are in degrees. */
#define BACK_UP_DISTANCE_MM       120
#define CLEAR_DISTANCE_MM         300
#define AVOID_TURN_DEG             80
#define HEADING_TOLERANCE_DEG       5

#define DRIVE_SPEED_MM_S          200
#define REVERSE_SPEED_MM_S        120
#define TURN_SPEED_MM_S           110
#define REORIENT_MIN_SPEED_MM_S    20
#define REORIENT_GAIN_MM_S_PER_DEG  3

static int16_t limitSpeed(const int16_t requestedSpeed,
    const int16_t maxWheelSpeed)
{
    if (maxWheelSpeed <= 0) {
        return 0;
    }
    return (requestedSpeed < maxWheelSpeed)
        ? requestedSpeed : maxWheelSpeed;
}

/* Positive error means the robot is left of the target heading. */
static int32_t signedAngleError(const int32_t currentAngle,
    const int32_t targetAngle)
{
    int32_t error = (currentAngle % 360 - targetAngle % 360) % 360;

    if (error > 180) {
        error -= 360;
    }
    else if (error < -180) {
        error += 360;
    }
    return error;
}

void KobukiNavigationStatechart(
    const int16_t maxWheelSpeed,
    const int32_t netDistance,
    const int32_t netAngle,
    const KobukiSensors_t sensors,
    const accelerometer_t accelAxes,
    int16_t * const pRightWheelSpeed,
    int16_t * const pLeftWheelSpeed,
    const bool isSimulator)
{
    static robotState_t state = INITIAL;
    static robotState_t unpausedState = DRIVE;
    static turnDirection_t avoidDirection = TURN_RIGHT;

    static int32_t groundAngle = 0;
    static int32_t distanceAtManeuverStart = 0;
    static int32_t angleAtManeuverStart = 0;

    /* Alternating the direction for a centre/both-side hit helps prevent
       repeatedly choosing the same trapped route. */
    static bool centreTurnRight = true;

    int16_t leftWheelSpeed = 0;
    int16_t rightWheelSpeed = 0;
    int16_t driveSpeed;
    int16_t reverseSpeed;
    int16_t turnSpeed;
    int32_t headingError;
    int32_t reorientSpeed;

    bool leftHazard;
    bool centreHazard;
    bool rightHazard;
    bool anyHazard;
    bool turnCompleted;

    /* accelAxes is available for extensions but is not required here. */
    (void)accelAxes;

    driveSpeed = limitSpeed(DRIVE_SPEED_MM_S, maxWheelSpeed);
    reverseSpeed = limitSpeed(REVERSE_SPEED_MM_S, maxWheelSpeed);
    turnSpeed = limitSpeed(TURN_SPEED_MM_S, maxWheelSpeed);

    /* Task 2 uses only the three front bumper sensors. Cliff and
       wheel-drop sensing are intentionally not part of this controller. */
    leftHazard = sensors.bumps_wheelDrops.bumpLeft;
    centreHazard = sensors.bumps_wheelDrops.bumpCenter;
    rightHazard = sensors.bumps_wheelDrops.bumpRight;

    anyHazard = leftHazard || centreHazard || rightHazard;

    /**********************************************************************
     * Pause region - highest priority, so B0 stops the robot immediately.
     **********************************************************************/
    if (state == INITIAL
        || state == PAUSE_WAIT_BUTTON_RELEASE
        || state == UNPAUSE_WAIT_BUTTON_PRESS
        || state == UNPAUSE_WAIT_BUTTON_RELEASE
        || sensors.buttons.B0) {

        switch (state) {
        case INITIAL:
            /* The robot is stationary here, so this is the direction in
               which it will first run. Pausing later does not change it. */
            groundAngle = netAngle;
            unpausedState = DRIVE;
            state = UNPAUSE_WAIT_BUTTON_PRESS;
            break;

        case PAUSE_WAIT_BUTTON_RELEASE:
            if (!sensors.buttons.B0) {
                state = UNPAUSE_WAIT_BUTTON_PRESS;
            }
            break;

        case UNPAUSE_WAIT_BUTTON_PRESS:
            if (sensors.buttons.B0) {
                state = UNPAUSE_WAIT_BUTTON_RELEASE;
            }
            break;

        case UNPAUSE_WAIT_BUTTON_RELEASE:
            if (!sensors.buttons.B0) {
                state = unpausedState;
            }
            break;

        default:
            /* A press while running records the interrupted state. */
            unpausedState = state;
            state = PAUSE_WAIT_BUTTON_RELEASE;
            break;
        }
    }

    /**********************************************************************
     * Running region.
     * New hazards have priority over normal maneuver-completion guards.
     * Check this region on the same call that resumes from pause.
     * During a recovery, wait until the turn finishes before accepting
     * another bumper hit, then back up again if contact persists.
     **********************************************************************/
    if (state >= DRIVE) {
        headingError = signedAngleError(netAngle, groundAngle);
        turnCompleted = state == TURN_AWAY
            && abs(signedAngleError(netAngle, angleAtManeuverStart))
                >= AVOID_TURN_DEG;

        if (anyHazard && ((state != BACK_UP && state != TURN_AWAY)
                || turnCompleted)) {
            if (leftHazard && !rightHazard) {
                avoidDirection = TURN_RIGHT;
            }
            else if (rightHazard && !leftHazard) {
                avoidDirection = TURN_LEFT;
            }
            else {
                avoidDirection = centreTurnRight ? TURN_RIGHT : TURN_LEFT;
                centreTurnRight = !centreTurnRight;
            }

            distanceAtManeuverStart = netDistance;
            state = BACK_UP;
        }
        else if (state == BACK_UP
            && abs(netDistance - distanceAtManeuverStart)
                >= BACK_UP_DISTANCE_MM) {

            angleAtManeuverStart = netAngle;
            state = TURN_AWAY;
        }
        else if (turnCompleted) {
            distanceAtManeuverStart = netDistance;
            state = CLEAR_OBSTACLE;
        }
        else if (state == CLEAR_OBSTACLE
            && abs(netDistance - distanceAtManeuverStart)
                >= CLEAR_DISTANCE_MM) {

            state = REORIENT;
        }
        else if (state == REORIENT
            && abs(headingError) <= HEADING_TOLERANCE_DEG) {

            state = DRIVE;
        }
    }

    /**********************************************************************
     * State actions - every state defines both wheel speeds.
     **********************************************************************/
    switch (state) {
    case INITIAL:
    case PAUSE_WAIT_BUTTON_RELEASE:
    case UNPAUSE_WAIT_BUTTON_PRESS:
    case UNPAUSE_WAIT_BUTTON_RELEASE:
        leftWheelSpeed = 0;
        rightWheelSpeed = 0;
        break;

    case DRIVE:
        leftWheelSpeed = driveSpeed;
        rightWheelSpeed = driveSpeed;
        break;

    case BACK_UP:
        leftWheelSpeed = -reverseSpeed;
        rightWheelSpeed = -reverseSpeed;
        break;

    case TURN_AWAY:
        if (avoidDirection == TURN_RIGHT) {
            leftWheelSpeed = turnSpeed;
            rightWheelSpeed = -turnSpeed;
        }
        else {
            leftWheelSpeed = -turnSpeed;
            rightWheelSpeed = turnSpeed;
        }
        break;

    case CLEAR_OBSTACLE:
        leftWheelSpeed = driveSpeed;
        rightWheelSpeed = driveSpeed;
        break;

    case REORIENT:
        /* Follow the shortest heading error, including after overshoot.
           Slow near the target so faster cruising does not worsen settling. */
        headingError = signedAngleError(netAngle, groundAngle);
        if (abs(headingError) <= HEADING_TOLERANCE_DEG) {
            leftWheelSpeed = 0;
            rightWheelSpeed = 0;
            break;
        }
        reorientSpeed = abs(headingError) * REORIENT_GAIN_MM_S_PER_DEG;
        if (reorientSpeed < REORIENT_MIN_SPEED_MM_S) {
            reorientSpeed = REORIENT_MIN_SPEED_MM_S;
        }
        if (reorientSpeed > turnSpeed) {
            reorientSpeed = turnSpeed;
        }
        leftWheelSpeed = (int16_t)((headingError > 0)
            ? reorientSpeed : -reorientSpeed);
        rightWheelSpeed = -leftWheelSpeed;
        break;

    default:
        leftWheelSpeed = 0;
        rightWheelSpeed = 0;
        break;
    }

    *pLeftWheelSpeed = leftWheelSpeed;
    *pRightWheelSpeed = rightWheelSpeed;

    /* Kept to document that simulator/hardware share the same behaviour. */
    (void)isSimulator;
}
