#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S2,     irFront,        sensorI2CCustom)
#pragma config(Sensor, S4,     irRight,        sensorI2CCustom)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     mLiftL,        tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     mLeft,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     mLiftR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     mRight,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     mIntake,       tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C4_1,    sHook,                tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    sBackboard,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "HTIRS Driver.h"
#include "Core Library.c"

const int IR_THRESHOLD = 10;

/**
 * Returns true if the AC IR sensor receiver of the given direction is above IR_THRESHOLD
 *
 * Direction is 0 for farthest left, 4 for farthest right.
 */
bool isIrInDir(int irSensor, int receiverDir)
{
	int readings[5];
	HTIRS2readAllACStrength(irSensor, readings[0], readings[1], readings[2], readings[3], readings[4]);

	return readings[receiverDir] >= IR_THRESHOLD;
}

task main()
{
	servo[sHook] = HOOK_UP;
	servo[sBackboard] = BACKBOARD_MAX;
	nMotorEncoder[mLiftL] = 0;

	waitForStartWithDelay();

	prepareSlide();

	Forward(500, 20);
	Forward(3000, 35);

	wait10Msec(50);

	const bool goalPointingForward = isIrInDir(irFront, 2);

	Backward(500, 20);
	Backward(500, 35);

	if (!goalPointingForward) { // if goal is pointing horizontal or diagonal
		wait10Msec(50);
		// Turn 45 to get second reading
		Turn(1250, 35, 1);

		wait10Msec(50);
		const bool goalPointingDiagonal = isIrInDir(irFront, 3);

		if (!goalPointingDiagonal) { // if goal is pointing horizontal
			// Move at diagonal towards goal
			Forward(500, 20);
			Forward(4500, 35);

			// Turn 45
			wait10Msec(50);
			Turn(1150, 50, -1);

			// Move in line with goal
			wait10Msec(50);
			Forward(1000, 20);

			// Turn to face goal
			wait10Msec(50);
			Turn(2000, 50, -1);

			// Lift and go up to goal
			wait10Msec(50);
			liftSlide(LIFT_MAX);
			Forward(700, 20);
		} else { // if goal is pointing diagonal
			Forward(500, 20);
			Forward(2150, 35);

			wait10Msec(50);
			Turn(2450, 35, -1);

			liftSlide(LIFT_MAX);

			Forward(700, 20);
		}
	} else { // if goal is pointing forward
		wait10Msec(50);
		liftSlide(LIFT_MAX);

		Forward(1600, 20);
	}

	motor[mIntake] = -100;
	wait10Msec(200);
	motor[mIntake] = 0;

	Backward(1600, 20);

	liftSlide(LIFT_MIN);
}
