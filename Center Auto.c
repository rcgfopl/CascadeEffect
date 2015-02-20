#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S3,     sensIR,         sensorI2CCustom)
#pragma config(Sensor, S4,     sensUS,         sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     mBackLeft,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     mFrontLeft,    tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     mIntake,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     mLiftR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     mFrontRight,   tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     mBackRight,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     mKnocker,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     mLiftL,        tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C2_1,    sFloodGate,           tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C2_2,    sIR,                  tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    sTongue,              tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "HTIRS Driver.h"
#include "Core Library.c"

task main()
{
	homeServos();
	servo[sIR] = SERVO_IR_EDGE_1;

	tenseSlide();
	nMotorEncoder[mLiftR] = 0;

	waitForStartWithDelay();

	// Drive away from starting position along wall
	straight(DIR_FORWARD, 60, 800);
	dispenseMomentum();

	int ac1, ac2, ac3, ac4, ac5;
	HTIRS2readAllACStrength(sensIR, ac1, ac2, ac3, ac4, ac5);

	if (ac3 <= 5) {
		// position 2 or 3

		servo[sIR] = SERVO_IR_EDGE_23;

		strafe(DIR_LEFT, 60, 1200);
		dispenseMomentum();

		wait10Msec(50);

		HTIRS2readAllACStrength(sensIR, ac1, ac2, ac3, ac4, ac5);
		servo[sIR] = SERVO_IR_FORWARD;

		if (ac3 <= 5) {
			// position 3

			strafe(DIR_LEFT, 60, 300);
			dispenseMomentum();

			straight(DIR_FORWARD, 60, 1100);
			dispenseMomentum();

			rotate(DIR_CLOCKWISE, 60, 850);
			dispenseMomentum();

			liftSlide(LIFT_MAX);

			straight(DIR_FORWARD, 25, 100);
			dispenseMomentum();
		} else {
			// position 2

			//straight(DIR_FORWARD, 60, 300);
			straight(DIR_FORWARD, 60, 200);
			dispenseMomentum();

			rotate(DIR_CLOCKWISE, 60, 450);
			dispenseMomentum();

			//straight(DIR_BACKWARD, 60, 140); // guess
			//dispenseMomentum()

			strafe(DIR_RIGHT, 60, 80);
			dispenseMomentum();

			liftSlide(LIFT_MAX);

			//straight(DIR_FORWARD, 25, 130);
			//dispenseMomentum();
		}
	} else {
		// position 1
	}

	// Get close enough
	lineupDistance();
	dispenseMomentum();

	// Center onto the goal
	lineupLeftRight();
	dispenseMomentum();

	// Raise the slide
	liftSlide(LIFT_MAX);

	// Dispense the ball
	servo[sTongue] = TONGUE_MIN;
	wait10Msec(200);
	servo[sTongue] = TONGUE_MAX;

	// Lower the slide
	liftSlide(LIFT_MIN);
	tenseSlide();

	// Strafe to the right, line up with the bar
	strafe(DIR_RIGHT, 60, 525);
	dispenseMomentum();

	// Ram into the bar to knock it down
	straight(DIR_FORWARD, 100, 2400);
	dispenseMomentum();
}
