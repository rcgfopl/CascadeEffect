#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S2,     IR,             sensorHiTechnicIRSeeker1200)
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
#pragma config(Servo,  srvo_S1_C4_2,    sTrapdoor,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "Core Library.c"

task main()
{
	servo[sHook] = HOOK_UP;
	nMotorEncoder[mLiftL] = 0;

	waitForStartWithDelay();

	prepareSlide();

	Forward(500, 20);
	Forward(2000, 35);

	wait10Msec(50);
	int sensorReading = SensorValue[IR];

	if (sensorReading == 0) { // if goal is south or southeast
		Turn(1250, 35, -1);

		wait10Msec(50);
		sensorReading = SensorValue[IR];

		if (sensorReading == 0 || sensorReading == 9) { // if goal is south
			Forward(500, 20);
			Forward(3500, 35);
			
			wait10Msec(50);
			Turn(1150, 50, 1);

			wait10Msec(50);
			Forward(2000, 20);

			wait10Msec(50);
			Turn(2450, 50, 1);

			wait10Msec(50);
			liftSlide(LIFT_MAX);
			Forward(900, 20);
		} else { // if goal is southeast
			Forward(500, 20);
			Forward(2300, 35);

			wait10Msec(50);
			Turn(2250, 50, 1);

			liftSlide(LIFT_MAX);

			Forward(750, 20);
		}
	} else { // if goal is east
		liftSlide(LIFT_MAX);

		Forward(1600, 20);
	}

	motor[mIntake] = -100;
	wait10Msec(200);
}