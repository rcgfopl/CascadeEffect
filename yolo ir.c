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
#include "HTIRS Driver.h"
#include "Core Library.c"
#include "Log Library.c"

task main()
{
	bDisplayDiagnostics = false;

	startLogs("yolo.txt", 3);
	addLog("jimmy", 31, 29, 384, 49, 20);
	addLog("geo", 31, 29, 384, 49, 20);
	addLog("KAPA_", 31, 29, 384, 49, 20);
	finishLogs();

	while (true) {
		int dir = HTIRS2readACDir(sensIR);

		int dc1, dc2, dc3, dc4, dc5;
		HTIRS2readAllDCStrength(sensIR, dc1, dc2, dc3, dc4, dc5);

		int ac1, ac2, ac3, ac4, ac5;
		HTIRS2readAllACStrength(sensIR, ac1, ac2, ac3, ac4, ac5);

		eraseDisplay();
		nxtDisplayTextLine(2, "Dir: %d", dir);
		nxtDisplayTextLine(3, "1: DC %2d AC %2d", dc1, ac1);
		nxtDisplayTextLine(4, "2: DC %2d AC %2d", dc2, ac2);
		nxtDisplayTextLine(5, "3: DC %2d AC %2d", dc3, ac3);
		nxtDisplayTextLine(6, "4: DC %2d AC %2d", dc4, ac4);
		nxtDisplayTextLine(7, "5: DC %2d AC %2d", dc5, ac5);

		wait10Msec(1);
	}
}
