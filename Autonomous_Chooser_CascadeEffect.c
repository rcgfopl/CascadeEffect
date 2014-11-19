#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     Ultra,          sensorSONAR)
#pragma config(Sensor, S3,     Ultra2,         sensorSONAR)
#pragma config(Sensor, S4,     IR,             sensorHiTechnicIRSeeker600)
#pragma config(Motor,  motorA,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     mLiftL,        tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     mLeft,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     mLiftR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     mRight,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     mIntake,       tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C4_1,    sHook,                tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    sTrapdoor,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define left 1
#define right -1

// whether the user has finished selecting a delay time
bool timeSelected = false;
// whether the robot will wait for the FCS to start (disable when testing)
bool startSelected = false;
// whether the user has selected to delay at the beginning of autonomous
bool waitSelected = true;
// how many seconds to delay before starting autonomous (used for coordinating with
// other teams)
int waitTime = 0;
// the message used by the wait prompt
string wait = "Yes";
//holds which  autonomous program we will be running
int programCounter = 1;
//holds the  maximum amount of autonomous programs we can run
int max = 6;

// standard library for talking to Samantha FCS
#include "JoystickDriver.c"
// our team-built function library
#include "FunctionLibrary_CascadeEffect.c"
//our team-built Autonomous library
#include "Autonomous_Library_CascadeEffect.c"

task main()
{
	nMotorEncoder[mLift1] = 0;
	nMotorEncoder[mRight] = 0;
	 selectTime();
	 startSelect();
	 while(true)
	 {
	   picker();
	  if(programCounter == 1)
		{
	    nxtDisplayCenteredTextLine(2, "Regular Autonomous");
	    if(nNxtButtonPressed == 3)
	    {
	        PlaySound(soundBlip);
	        if(waitSelected){waitForStart();}
	        wait1Msec(waitTime*1000);

	       //insert autonomous program here

	        wait1Msec(500);
	    }
	  }
	  else if(programCounter == 2)
	  {
	    nxtDisplayCenteredTextLine(2, "FA to FR");
	    if(nNxtButtonPressed == 3)
	    {
	        PlaySound(soundBlip);
	        if(waitSelected){waitForStart();}
	        wait1Msec(waitTime*1000);

	       //insert autonomous program here

	        wait1Msec(500);
	    }
	  }
	  else if(programCounter == 3)
	  {
	    nxtDisplayCenteredTextLine(2, "FA to BR");
	    if(nNxtButtonPressed == 3)
	    {
	        PlaySound(soundBlip);
	        if(waitSelected){waitForStart();}
	        wait1Msec(waitTime*1000);

	        //insert autonomous program here

	        wait1Msec(500);
	    }
	  }
	  else if(programCounter == 4)
	  {
	    nxtDisplayCenteredTextLine(2, "Hardcoded Autonomous()");
	    if(nNxtButtonPressed == 3)
	    {
	        PlaySound(soundBlip);
	        if(waitSelected){waitForStart();}
	        wait1Msec(waitTime*1000);
	        //insert autonomous program here
	        wait1Msec(500);
	    }
	  }
	  else if(programCounter  == 5)
	  {
	  	nxtDisplayCenteredTextLine(2, "Hardcoded Reverse");
	  	if(nNxtButtonPressed == 3)
	  	{
	  		PlaySound(soundBlip);
	  		if(waitSelected){waitForStart();}
	  		wait1Msec(waitTime * 1000);
	  		//insert autonomous program here
	  		wait1Msec(500);
	  	}
	  }
	  else if(programCounter == 6)
	  {
	    nxtDisplayCenteredTextLine(2, "New IR");
	    if(nNxtButtonPressed == 3)
	    {
	        PlaySound(soundBlip);
	        if(waitSelected){waitForStart();}
	        wait1Msec(waitTime*1000);
	        //insert autonomous program here
	        wait1Msec(500);
	    }
	  }
	}
}
