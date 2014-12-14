#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S2,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S4,     IR2,            sensorHiTechnicIRSeeker1200)
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

const int ADDITIONAL_DIST = 0;
task main()
{
	//Do we want waitForStart() to be enabled, and do we want a delay before the program starts after waitForStart is completed?
	//we decide this right here.
	waitForStartWithDelay();
	//Goes forward a tiny bit so we can get an accurate sensor value.
	Forward(200,25);
	Forward(300,45);
	wait1Msec(500);
	Turn(450,50,Left);
/*	//checks for the sensor value at this point
	if(SensorValue(IR) == 5)
	{
		//In this if statement, we run the code if we get a '5' as our sensor value.
		//this means that the beacon and the high goal are right in front of us.

		//Raises the lift to its highest point
		liftSlide(LIFT_MAX);
		wait1Msec(500);
		//goes forward a tiny bit more so that dispensing the ball would put it right into the goal
		Forward(300,100);
		wait1Msec(500);
		//Dispenses the ball
		motor[mIntake] = 100;
		wait1Msec(1000);
		//Goes back from the goal
		Backward(500,100);
		wait1Msec(500);
		//Brings the lift down to its lowest point
		liftSlide(LIFT_MIN);
		wait1Msec(500);
	}
	else
	{
		//This is run if the high goal and beacon are not in the position immediately ahead of us.

		//Turns right so that we can go in a circle around the ramp
		Turn(450,100,Left);
		wait1Msec(500);
		//Goes in a circle around the ramp until the point where the beacon is in the '5' range for the second IR sensor
		while(SensorValue(IR2) != 5)
		{
			moveCircle(100,100);
		}
		wait1Msec(500);
		//Goes in a circle for any additional distance needed to put the robot right in front of the beacon.
		moveCircle(100,ADDITIONAL_DIST);
		wait1Msec(500);
		//Turns to face the beacon
		Turn(450,100,Left);
		wait1Msec(500);
		//Raises the lift to its highest point
		liftSlide(LIFT_MAX);
		wait1Msec(500);
		//goes forward a tiny bit more so that dispensing the ball would put it right into the goal
		Forward(300,100);
		wait1Msec(500);
		//Dispenses the ball
		motor[mIntake] = 100;
		wait1Msec(1000);
		//Goes back from the goal
		Backward(500,100);
		wait1Msec(500);
		//Brings the lift down to its lowest point
		liftSlide(LIFT_MIN);
		wait1Msec(500);
	}*/
}