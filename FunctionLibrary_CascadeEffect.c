#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"

// Direction representing a mLeft (counter-clockwise) turn
// This is used as a parameter for turning functions.
#define Left 1
// Direction representing a mRight (clockwise) turn
// This is used as a parameter for turning functions.
#define Right -1
// The distance between the left and right wheels on the robot, in feet
const float ROBOT_WIDTH = 0;
// The radius of the circle around the goal at which the robot should orbit, in feet
const float GOAL_ORBIT_RADIUS = 0;

// When the joystick is within this value of zero, it will clamp to zero
const int JOY_THRESHOLD = 8;

//Miscellaneous methods

/*
// Allows us to pick our choice of autonomous program.
// Uses a GUI and the NXT buttons.
int picker()
{
	while(true)
	{
		wait1Msec(500);
		int x = 1;
		int y = 0;
		if(y == 0)
		{
			while(nNxtButtonPressed != 3)
			{
				nxtDisplayCenteredTextLine(2,"Hello There! This is the Autonomous Chooser.");
			 	nxtDisplayCenteredBigTextLine(4,"Please pick the beacon you would like to go to.");
	   	}
	   	while(nNxtButtonPressed == 3) {}
	   	eraseDisplay();
	   	wait1Msec(500);
	   	y++;
	 	}
	  while (y > 0)
	  {
			nxtDisplayCenteredTextLine(2,"%d",x);
			if(nNxtButtonPressed == 1)
			{
				wait1Msec(500);
				if(x == max)
					{x=1;}
				else
		 			{x++;}
			}
			else if(nNxtButtonPressed == 2)
			{
				wait1Msec(500);
				if(x == 1)
					{x=max;}
				else
		 			{x--;}
			}
			else if(nNxtButtonPressed == 3)
			{
				wait1Msec(500);
		 		return x;
			}
			eraseDisplay();
		}
	}
	eraseDisplay();
	nxtDisplayCenteredTextLine(2, "Oh my. Why did we get here? Something went wrong.");
	wait1Msec(100);
	return 1;

}
*/

/*
// allows us to pick the time we wait before our autonomous program starts.
void selectTime()
{
	while(!timeSelected)
	{
		nxtDisplayCenteredTextLine(2, "Time: %2.1f", (waitTime));
		if (nNxtButtonPressed == 1)
		{
			wait1Msec(500);
			PlaySound(soundBlip);
			waitTime += 2.5;
		}
		else if (nNxtButtonPressed == 2)
		{
			wait1Msec(500);
			PlaySound(soundBlip);
			waitTime -= 2.5;
		}
		if (waitTime > 15)
		{
			waitTime = 15;
		}
		else if (waitTime < 0)
		{
			waitTime = 0;
		}
		if(nNxtButtonPressed == 3)
		{
			wait1Msec(500);
			timeSelected = true;
		}
	}
}
*/

/*
// allows us to pick if we want WaitForStart method to run.
void startSelect()
{
	while(!startSelected)
	{
		nxtDisplayCenteredTextLine(2, "WaitForStart? %s", wait);
		if (nNxtButtonPressed == 1)
		{
			wait1Msec(500);
			PlaySound(soundBlip);
			wait = "Yes";
			waitSelected = true;
		}
		else if (nNxtButtonPressed == 2)
		{
			wait1Msec(500);
			PlaySound(soundBlip);
			wait = "No";
			waitSelected = false;
		}
		if(nNxtButtonPressed == 3)
		{
			wait1Msec(500);
			startSelected = true;
		}
	}
}
*/

//various methods to move forward.

// Move forward based on encoders
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void Forward(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight]) < distance)
	{
		motor[mLeft] = power;
		motor[mRight] = power;
	}
  	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Move forward based on encoders while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// other robot moves away.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void ForwardWithSonar(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight]) < distance)
  {
		int value = SensorValue[Ultra];
		if((value>=255 || value<0))
		{
			motor[mLeft] = power;
			motor[mRight] = power;
		}
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Move forward based on encoders while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// Uses both the ultrasonic sensors in parallel.
// other robot moves away.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void ForwardWithTwoSonar(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight]) < distance)
  {
		int value  = USreadDist(Ultra);
		int value2 = USreadDist(Ultra2);
		if((value>=255 || value<0) || (value2>=255 || value<0))
		{
			motor[mLeft] = power;
			motor[mRight] = power;
		}
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Move forward based on time while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// Uses both the ultrasonic sensors in parallel.
// other robot moves away.
// time: the number of milliseconds to move for
// power: the power level the motors will be driven at
void ForwardWithTimeUS(int time, int power)
{
	unsigned long endTime = nPgmTime + time;
	while(nPgmTime < endTime)
	{
		int mLeftUS = USreadDist (Ultra);
		int mRightUS = USreadDist (Ultra2);
		if ((mLeftUS > 0 && mLeftUS < 255) || (mRightUS > 0 && mRightUS < 255))
		{
				motor[mRight] = 0;
				motor[mLeft] = 0;
		}
		else
		{
				motor[mRight] = power;
				motor[mLeft] = power;
		}
	}
	motor[mRight] = 0;
	motor[mLeft] = 0;
}

// Move forward based on encoders
// time: the number of milliseconds to move for
// power: the power level the motors will be driven at
void ForwardWithTime(int time, int power)
{
	unsigned long endTime = nPgmTime + time;
	while(nPgmTime < endTime)
	{
				motor[mRight] = power;
				motor[mLeft] = power;
	}
	motor[mRight] = 0;
	motor[mLeft] = 0;
}

// Move forward based on encoders
// Compensates for a weaker mRight side of the drive train.
// time: the number of milliseconds to move for
// power: the power level the motors will be driven at
void ForwardWithModTime(int time, int power)
{
	unsigned long endTime = nPgmTime + time;
	while(nPgmTime < endTime)
	{
				motor[mRight] = power + 15;
				motor[mLeft] = power;
	}
	motor[mRight] = 0;
	motor[mLeft] = 0;
}

// Move forward based on encoders
// Compensates for a weaker mRight side of the drive train.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void ModForward(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight]) < distance)
	{
		motor[mLeft] = power + 25;
		motor[mRight] = power;
	}
  	motor[mLeft] = 0;
	motor[mRight] = 0;
}

//various methods to move backwards.

// Move backward based on encoders
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void Backward(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight] )> -distance)
	{
		motor[mLeft] = -power;
		motor[mRight] = -power;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Move backward based on encoders while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// other robot moves away.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void BackwardWithSonar(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight] )> -distance)
	{
		int value = SensorValue[Ultra];
		if(value>=255 || value<0)
		{
			motor[mLeft] = -power;
			motor[mRight] = -power;
		}
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Move backward based on encoders while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// Uses both the ultrasonic sensors in parallel.
// other robot moves away.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void BackwardWithTwoSonar(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight]) < distance)
  {
		int value  = USreadDist(Ultra);
		int value2 = USreadDist(Ultra2);
		if((value>=255 || value<0) || (value2>=255 || value<0))
		{
			motor[mLeft] = -power;
			motor[mRight] = -power;
		}
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Move backward based on time while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// Uses both the ultrasonic sensors in parallel.
// other robot moves away.
// time: the number of milliseconds to move for
// power: the power level the motors will be driven at
void BackwardWithTimeUS(int time, int power)
{
	unsigned long endTime = nPgmTime + time;
	while(nPgmTime < endTime)
	{
		int mLeftUS = USreadDist (Ultra);
		int mRightUS = USreadDist (Ultra2);
		if ((mLeftUS > 0 || mLeftUS < 255) || (mRightUS > 0 || mRightUS < 255))
		{
				motor[mRight] = -power;
				motor[mLeft] = -power;
		}
		else
		{
				motor[mRight] = -power;
				motor[mLeft] = -power;
		}
	}
	motor[mRight] = 0;
	motor[mLeft] = 0;
}

// Move backward based on encoders
// time: the number of milliseconds to move for
// power: the power level the motors will be driven at
void BackwardWithTime(int time, int power)
{
	unsigned long endTime = nPgmTime + time;
	while(nPgmTime < endTime)
	{
				motor[mRight] = -power;
				motor[mLeft] = -power;
	}
	motor[mRight] = 0;
	motor[mLeft] = 0;
}

// Move backward based on encoders
// Compensates for a weaker mRight side of the drive train.
// time: the number of milliseconds to move for
// power: the power level the motors will be driven at
void BackwardWithModTime(int time, int power)
{
	unsigned long endTime = nPgmTime + time;
	while(nPgmTime < endTime)
	{
				motor[mRight] = -power - 15;
				motor[mLeft] = -power;
	}
	motor[mRight] = 0;
	motor[mLeft] = 0;
}

// Move backward based on encoders
// Compensates for a weaker mRight side of the drive train.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void ModBackward(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while((nMotorEncoder[mRight] )> -distance)
	{
		motor[mLeft] = -power-25;
		motor[mRight] = -power;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

//a couple methods to turn.

// Turn based on encoders
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
// dir: 1 for mLeft, -1 for mRight
void Turn(int distance, int power, int dir)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while(abs(nMotorEncoder[mRight]) < distance)
	{
		motor[mLeft] = power*dir;
		motor[mRight] = -power*dir;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

// Turn based on encoders
// Compensates for a weaker mRight side of the drive train.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
// dir: 1 for mLeft, -1 for mRight
void ModTurn(int distance, int power, int dir)
{
	nMotorEncoder[mLeft] = 0;
	nMotorEncoder[mRight] = 0;

	while(abs(nMotorEncoder[mRight]) < distance)
	{
		motor[mLeft] = power*dir + 20 * dir;
		motor[mRight] = -power*dir;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

//moves the robot in a circle around a certain point.
//To move backwards, maxPwr should be a negative number.
void moveCircle(int maxPwr, int dist)
{
	//a is the smaller distance (distance from left wheels to center)
	//b is the larger distance (distance from right wheels to center)
	float a = GOAL_ORBIT_RADIUS, b = a+ROBOT_WIDTH;
	//finds the ratio that will help us determine the power level for each side
	float ratio = a/b;
	while(nMotorEncoder(Right) < dist)
	{
		motor[mLeft]=(int)(maxPwr*ratio + 0.5);
		motor[mRight]= maxPwr;
	}
}

int threshold(int value)
{
	if (abs(value) < JOY_THRESHOLD) {
		return 0;
	} else {
		return value;
	}
}
