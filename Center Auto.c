#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     sensIR,         sensorI2CCustom)
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
	straight(DIR_FORWARD, 45, 1400);
	dispenseMomentum();

	int ac1, ac2, ac3, ac4, ac5;
	HTIRS2readAllACStrength(sensIR, ac1, ac2, ac3, ac4, ac5);

	if (ac3 <= 5) {
		// position 2 or 3

		servo[sIR] = SERVO_IR_EDGE_23;

		strafe(DIR_LEFT, 45, 1200);
		dispenseMomentum();

		wait10Msec(50);

		HTIRS2readAllACStrength(sensIR, ac1, ac2, ac3, ac4, ac5);
		servo[sIR] = SERVO_IR_FORWARD;

		if (ac3 <= 5) {
			// position 3

			strafe(DIR_LEFT, 45, 300);
			dispenseMomentum();

			straight(DIR_FORWARD, 45, 1100);
			dispenseMomentum();

			rotate(DIR_CLOCKWISE, 45, 850);
			dispenseMomentum();

			liftSlide(LIFT_MAX);

			straight(DIR_FORWARD, 25, 100);
			dispenseMomentum();
		} else {
			// position 2

			//straight(DIR_FORWARD, 45, 300);
		  straight(DIR_FORWARD, 45, 200);
			dispenseMomentum();

			rotate(DIR_CLOCKWISE, 45, 450);
			dispenseMomentum();

			//straight(DIR_BACKWARD, 45, 140); // guess
			//dispenseMomentum()

			strafe(DIR_RIGHT, 45, 80);
			dispenseMomentum();

			liftSlide(LIFT_MAX);

			//straight(DIR_FORWARD, 25, 130);
			//dispenseMomentum();
		}
	} else {
		// position 1

		liftSlide(LIFT_MAX);

		straight(DIR_FORWARD, 25, 120);
		dispenseMomentum();
	}

	// Dispense the ball
	servo[sTongue] = TONGUE_MIN;
	wait10Msec(200);
	servo[sTongue] = TONGUE_MAX;

	// Back away from the goal
	//straight(DIR_BACKWARD, 45, 500);
	//dispenseMomentum();

	// Lower the slide
	liftSlide(LIFT_MIN);
	tenseSlide();

	// Strafe to the right, line up with the bar
	strafe(DIR_RIGHT, 45, 680);
	dispenseMomentum();

	// Ram into the bar to knock it down
	straight(DIR_FORWARD, 100, 2400);
	dispenseMomentum();
}
