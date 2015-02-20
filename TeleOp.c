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
#pragma config(Servo,  srvo_S1_C2_4,    sGrabber,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "Core Library.c"

int intakeDir = 0;

bool frontIsBack = false;

bool driveActive, liftActive;

void drive()
{
	if (joy1Btn(11)) { // left joystick clicker
		frontIsBack = true;
		} else if (joy1Btn(12)) { // right joystick clicker
		frontIsBack = false;
	}

	int left = threshold(joystick.joy1_y1);
	int right = threshold(joystick.joy1_y2);
	int strafe = threshold(joystick.joy1_x1);
	int rotation = threshold(joystick.joy1_x2);

	if (abs(strafe) >= 99) {
		left = right = 0;
	} else {
		strafe = 0;
	}

	driveActive = left != 0 || right != 0 || strafe != 0 || rotation != 0;

	if (!joy1Btn(6)) { // left top shoulder
		left /= 4;
		right /= 4;
		strafe /= 2;
		rotation /= 4;
	}

	if (!frontIsBack) {
		yolodrive(left, right, strafe, rotation);
		} else {
		yolodrive(-right, -left, -strafe, rotation);
	}
}

void lift()
{
	int power = threshold(joystick.joy2_y1);

	if (joy2Btn(11)) { // press left joystick
		nMotorEncoder[mLiftR] = 0;
		} else {
		// Go slower when going down
		if (power < 0) {
			power = power * 40 / 100;
		}

		bool tooLow = power < 0 && nMotorEncoder[mLiftR] <= LIFT_MIN;
		bool tooHigh = power > 0 && nMotorEncoder[mLiftR] >= LIFT_MAX;
		if (tooLow || tooHigh) {
			power = 0;
		}
	}

	if (joy2Btn(5)) { // left top shoulder
		power /= 3;
	}

	liftActive = power != 0;

	motor[mLiftL] = motor[mLiftR] = power;
}

void tallGoal(){
	if(joy2Btn(6)){
		while(nMotorEncoder[mLiftR] < LIFT_TALL_GOAL){
			motor[mLiftL] = 100;
			motor[mLiftR] = 100;
		}
		motor[mLiftL] = 0;
		motor[mLiftR] = 0;
	}
}

void intake()
{
	if (joy2Btn(4)) {
		intakeDir = 1;
		} else if (joy2Btn(3)) {
		intakeDir = -1;
		} else if (joy2Btn(1)) {
		intakeDir = 0;
	}

	if (!driveActive || !liftActive) {
		motor[mIntake] = intakeDir * 100;
	} else {
		motor[mIntake] = 0;
	}
}

void tongue()
{
	if (joy2Btn(2)) {
		lowerTongue();
		} else {
		raiseTongue();
	}
}

void knocker()
{
	if (joystick.joy1_TopHat == 2) {
		motor[mKnocker] = 25;
		} else if (joystick.joy1_TopHat == 6) {
		motor[mKnocker] = -25;
		} else {
		motor[mKnocker] = 0;
	}
}

void floodgate()
{
	if(joy1Btn(7)){
		servo[sFloodGate] = 5;
	}
	else if(joy1Btn(5)){
		servo[sFloodGate] = 250;
	}
	else{
		servo[sFloodGate] = 127;
	}
}

void goalGrabber(){
	if(joy1Btn(4)){
		servo[sGrabber] = GRABBER_MIN;
	}
	else if(joy1Btn(2)){
		servo[sGrabber] = GRABBER_MAX;
	}
}

task main()
{
	nMotorEncoder[mLiftR] = 0;

	while(true) {
		getJoystickSettings(joystick);

		if (!bDisconnected) {
			drive();
			lift();
			intake();
			tallGoal();
			knocker();
			floodgate();
			tongue();
			goalGrabber();
		} else {
			motor[mFrontLeft] = motor[mBackLeft] = motor[mFrontRight] = motor[mBackRight] = motor[mLiftL] =
			motor[mLiftR] = motor[mIntake] = motor[mKnocker] = 0;
		}

		wait1Msec(100);
	}
}
