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

// Ask the user a question with two options, selected via the left and right buttons
//
// This returns -1 if the left option is chosen, and 1 if the right option is chosen.
int promptLeftRight(string prompt, string left, string right)
{
	int choice = 0;
	while (true) {
		nxtDisplayTextLine(1, prompt);

		string choiceText = "";
		if (choice == -1) {
			choiceText = left;
		} else if (choice == 1) {
			choiceText = right;
		}
		nxtDisplayTextLine(2, choiceText);

		nxtDisplayTextLine(4, "Enter to accept");

		while (nNxtButtonPressed != -1) {}
		while (nNxtButtonPressed == -1) {}

		if (nNxtButtonPressed == kLeftButton) {
			choice = -1;
		} else if (nNxtButtonPressed == kRightButton) {
			choice = 1;
		} else if (nNxtButtonPressed == kEnterButton && choice != 0) {
			return choice;
		}
	}

	return 0; // this never gets run, it just appeases the compiler
}

// Equivalent to waitForStart but provides the user an option to bypass by pressing the
// enter button
//
// This waits for no buttons to be pressed before starting the waiting.
void waitForStartOptional()
{
	while (nNxtButtonPressed != -1) {}
	while (joystick.StopPgm && nNxtButtonPressed != kEnterButton) {
		getJoystickSettings(joystick);
	}
}

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

// If the given value is within JOY_THRESHOLD of zero, clamp the value to zero
//
// This should be wrapped around joystick values, as in:
// motor[mLeft] = threshold(joystick.joy1_y1);
int threshold(int value)
{
	if (abs(value) < JOY_THRESHOLD) {
		return 0;
	} else {
		return value;
	}
}
