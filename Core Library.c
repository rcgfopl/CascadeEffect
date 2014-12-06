#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"

const int BACKBOARD_MAX = 240;
const int BACKBOARD_MIN = 80;

const int LIFT_MIN = 0;
const int LIFT_MAX = 7400;
const int LIFT_SLOW = 2100;

const int LIFT_THRESHOLD = 50;
const int LIFT_CUSHION = 3000;

const int HOOK_DOWN = 0;
const int HOOK_UP = 180;

// Direction representing a mLeft (counter-clockwise) turn
// This is used as a parameter for turning functions.
#define Left 1
// Direction representing a mRight (clockwise) turn
// This is used as a parameter for turning functions.
#define Right -1
// The distance between the left and right wheels on the robot, in feet
const float ROBOT_WIDTH = 42;
// The radius of the circle around the goal at which the robot should orbit, in feet
const float GOAL_ORBIT_RADIUS = 0;

// When the joystick is within this value of zero, it will clamp to zero
const int JOY_THRESHOLD = 8;

// Ask the user a question with two options, selected via the left and right buttons
//
// This returns -1 if the left option is chosen, and 1 if the right option is chosen.
int promptLeftRight(const string prompt, const string left, const string right)
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
}

// Ask the user to enter a whole, positive or zero number
int promptNumber(const string prompt)
{
	int result = 0;
	while (true) {
		eraseDisplay();
		nxtDisplayTextLine(1, prompt);

		if (result < 0) result = 0;

		nxtDisplayTextLine(3, "yolo%d", result);

		while (nNxtButtonPressed != -1) {}
		while (nNxtButtonPressed == -1) {}
		if (nNxtButtonPressed == kLeftButton) {
			--result;
		} else if (nNxtButtonPressed == kRightButton) {
			++result;
		} else if (nNxtButtonPressed == kEnterButton) {
			return result;
		}
	}
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

// Prompts the user for a delay, then proceeds to waitForStartOptional before performing the
// chosen delay
void waitForStartWithDelay()
{
	bDisplayDiagnostics = false;
	int delay = promptNumber("Delay (sec):");
	bDisplayDiagnostics = true;

	waitForStartOptional();

	wait10Msec(delay * 100);
}

//various methods to move forward.

// Move forward based on encoders
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void Forward(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;

	while((nMotorEncoder[mLeft]) < distance)
	{
		motor[mLeft] = power;
		motor[mRight] = power;
	}
  	motor[mLeft] = 0;
	motor[mRight] = 0;
}

/*
// Move forward based on encoders while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// other robot moves away.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void ForwardWithSonar(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;

	while((nMotorEncoder[mLeft]) < distance)
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

	while((nMotorEncoder[mLeft]) < distance)
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
*/

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

	while((nMotorEncoder[mLeft]) < distance)
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

	while(nMotorEncoder[mLeft] > -distance)
	{
		motor[mLeft] = -power;
		motor[mRight] = -power;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

/*
// Move backward based on encoders while avoiding collisions
// If the ultrasonic sensors see another robot, this will stop the robot until the
// other robot moves away.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
void BackwardWithSonar(int distance, int power)
{
	nMotorEncoder[mLeft] = 0;

	while((nMotorEncoder[mLeft] )> -distance)
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

	while((nMotorEncoder[mLeft]) < distance)
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
*/

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

	while((nMotorEncoder[mLeft] )> -distance)
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
// dir: 1 for left, -1 for right
void Turn(int distance, int power, int dir)
{
	nMotorEncoder[mLeft] = 0;

	while(abs(nMotorEncoder[mLeft]) < distance)
	{
		motor[mLeft] = -power*dir;
		motor[mRight] = power*dir;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}

/*
// Turn based on encoders
// Compensates for a weaker mRight side of the drive train.
// distance: the number of degrees to elapse on the encoders
// power: the power level the motors will be driven at
// dir: 1 for mLeft, -1 for mRight
void ModTurn(int distance, int power, int dir)
{
	nMotorEncoder[mLeft] = 0;

	while(abs(nMotorEncoder[mLeft]) < distance)
	{
		motor[mLeft] = power*dir + 20 * dir;
		motor[mRight] = -power*dir;
	}
	motor[mLeft] = 0;
	motor[mRight] = 0;
}
*/

//moves the robot in a circle around a certain point.
//To move backwards, maxPwr should be a negative number.
void moveCircle(int maxPwr, int dist)
{
	//a is the smaller distance (distance from left wheels to center)
	//b is the larger distance (distance from right wheels to center)
	float a = GOAL_ORBIT_RADIUS, b = a+ROBOT_WIDTH;
	//finds the ratio that will help us determine the power level for each side
	float ratio = a/b;
	nMotorEncoder(mRight) = nMotorEncoder(mLeft) = 0;
	while(nMotorEncoder(mRight) < dist || nMotorEncoder(mLeft) < dist)
	{
		motor[mLeft]=(int)(maxPwr*ratio + 0.5);
		motor[mRight]= maxPwr;
	}
}

// Lift or lower the slide to the given encoder target
//
// This will automatically use smart power values depending on the direction and
// height of the slide.
//
// The slide will be parked within LIFT_THRESHOLD of the given target.
void liftSlide(int target)
{
	while (true) {
		int distance = target - nMotorEncoder[mLiftL];
		if (abs(distance) <= LIFT_THRESHOLD) break;

		int power = 0;
		if (distance > 0) {
			power = 100;
		} else if (distance < 0) {
			if (nMotorEncoder[mLiftL] <= LIFT_SLOW) {
				power = -5;
			} else {
				power = -20;
			}
		}

		motor[mLiftL] = motor[mLiftR] = power;
	}

	motor[mLiftL] = motor[mLiftR] = 0;
}

// Lift the slide above zero so that it has a couple inches of room above the ground
// while driving (this lifts to LIFT_CUSHION)
//
// This does nothing if the slide is already above LIFT_CUSHION.
void prepareSlide()
{
	if (nMotorEncoder[mLiftL] < LIFT_CUSHION) {
		liftSlide(LIFT_CUSHION);
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
