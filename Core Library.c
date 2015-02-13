const int LIFT_MIN = 0;
const int LIFT_MAX = 7700;
const int LIFT_SLOW = 400;

const int TONGUE_MIN = 110;
const int TONGUE_MAX = 235;

const int LIFT_THRESHOLD = 50;
const int LIFT_CUSHION = 3000;

const int LIFT_TALL_GOAL = 5700;

const int SERVO_IR_FORWARD = 115;
const int SERVO_IR_EDGE_1 = 160;
const int SERVO_IR_EDGE_23 = /* TODO */0;

// Grabber is up
const int GRABBER_MIN = 10;
// Grabber is down
const int GRABBER_MAX = 220;
// Position at which the grabber won't be in the IR sensor's line of sight
const int GRABBER_UNBLOCK_IR = 80;

// Direction representing a mLeft (counter-clockwise) turn
// This is used as a parameter for turning functions.
#define TURN_LEFT 1
// Direction representing a mRight (clockwise) turn
// This is used as a parameter for turning functions.
#define TURN_RIGHT -1

// When the joystick is within this value of zero, it will clamp to zero
const int JOY_THRESHOLD = 60;

/**
 * Constants for autonomous movement
 */

#define DIR_FORWARD 1
#define DIR_BACKWARD -1

#define DIR_RIGHT 1
#define DIR_LEFT -1

#define DIR_CLOCKWISE 1
#define DIR_COUNTERCLOCKWISE -1

/*
 * Input
 */

// If the given value is within JOY_THRESHOLD of zero, clamp the value to zero
//
// This should be wrapped around joystick values, as in:
// motor[mLeft] = threshold(joystick.joy1_y1);
int threshold(int value)
{
  if (abs(value) < JOY_THRESHOLD) {
    return 0;
  } else {
    return value * 100 / 128;
  }
}

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

/*
 * Initialization
 */

/**
 * Set all servos to their initial, compact positions
 */
void homeServos()
{
	servo[sGrabber] = GRABBER_MIN;
	servo[sTongue] = TONGUE_MAX;
	servo[sIR] = SERVO_IR_FORWARD;
}

/*
 * Omnidrive locomotion
 */

/**
* Set motor powers for yolonomic drive
*
* left and right: the tank drive values from -100 to 100.
* strafe: the movement left and right from -100 to 100.
* rotation: the rotation from -100 to 100.
*/
void yolodrive(int left, int right, int strafe, int rotation)
{
	int pFL = left + strafe + rotation;
	int pBL = left - strafe + rotation;
	int pFR = right - strafe - rotation;
	int pBR = right + strafe - rotation;

	float biggest = abs(pFL);
	if (abs(pBL) > biggest) biggest = abs(pBL);
	if (abs(pFR) > biggest) biggest = abs(pFR);
	if (abs(pBR) > biggest) biggest = abs(pBR);

	float scale = 1;
	if (biggest > 100) {
		scale = 100.0 / biggest;
	}

	motor[mFrontLeft] = (int) (scale * pFL);
	motor[mBackLeft] = (int) (scale * pBL);
	motor[mFrontRight] = (int) (scale * pFR);
	motor[mBackRight] = (int) (scale * pBR);
}

/**
 * Get the average value of both drive encoders
 *
 * This uses absolute values, so it can be used for movements
 * even when the two motors are moving in opposite directions.
 */
int driveEncoderAverage()
{
	return (abs(nMotorEncoder[mFrontLeft]) + abs(nMotorEncoder[mFrontRight])) / 2;
}

/**
 * Move straight using yolodrive for the given encoder distance
 *
 * Power must be positive.
 * Direction must be either DIR_FORWARD or DIR_BACKWARD.
 */
void straight(int direction, int power, int distance)
{
	// The rate at which the robot should power up, in the form %power over milliseconds
	const float POWER_RATE = 100.0 / 2000;

	// The encoder distance from the end of a movement at which the robot should enter the slow zone
	const int DIST_FOR_SLOW = 600;

	// The maximum power at which the motors should move when the robot is in the slow zone
	const int SLOW_POWER = 20;

	nMotorEncoder[mFrontLeft] = 0;
	nMotorEncoder[mFrontRight] = 0;

	long startTime = nPgmTime;
	int timeToFull = (int) (power / POWER_RATE);

	while (true) {
		int elapsedTime = nPgmTime - startTime;
		int remainingDist = distance - driveEncoderAverage();

		if (remainingDist <= 0) break;

		int currentPower;
		if (elapsedTime < timeToFull) {
			currentPower = (int) (elapsedTime * POWER_RATE);
		} else {
			currentPower = power;
		}
		if (remainingDist <= DIST_FOR_SLOW && currentPower > SLOW_POWER) {
			currentPower = SLOW_POWER;
		}

		currentPower *= direction;
		yolodrive(currentPower, currentPower, 0, 0);

		wait10Msec(1);
	}

	yolodrive(0, 0, 0, 0);
}

/**
 * Strafe using yolodrive for the given encoder distance
 *
 * Power must be positive.
 * Direction must be either DIR_RIGHT or DIR_LEFT.
 */
void strafe(int direction, int power, int distance)
{
	// The rate at which the robot should power up, in the form %power over milliseconds
	const float POWER_RATE = 100.0 / 2000;

	// The encoder distance from the end of a movement at which the robot should enter the slow zone
	const int DIST_FOR_SLOW = 400;

	// The maximum power at which the motors should move when the robot is in the slow zone
	const int SLOW_POWER = 40;

	nMotorEncoder[mFrontLeft] = 0;
	nMotorEncoder[mFrontRight] = 0;

	long startTime = nPgmTime;
	int timeToFull = (int) (power / POWER_RATE);

	while (true) {
		int elapsedTime = nPgmTime - startTime;
		int remainingDist = distance - driveEncoderAverage();

		if (remainingDist <= 0) break;

		int currentPower;
		if (elapsedTime < timeToFull) {
			currentPower = (int) (elapsedTime * POWER_RATE);
		} else {
			currentPower = power;
		}
		if (remainingDist <= DIST_FOR_SLOW && currentPower > SLOW_POWER) {
			currentPower = SLOW_POWER;
		}

		currentPower *= direction;
		yolodrive(0, 0, currentPower, 0);

		wait10Msec(1);
	}

	yolodrive(0, 0, 0, 0);
}

/**
 * Rotate using yolodrive for the given encoder distance
 *
 * Power must be positive.
 * Direction must be either DIR_CLOCKWISE or DIR_COUNTERCLOCKWISE.
 */
void rotate(int direction, int power, int distance)
{
	// The rate at which the robot should power up, in the form %power over milliseconds
	const float POWER_RATE = 100.0 / 2000;

	// The encoder distance from the end of a movement at which the robot should enter the slow zone
	const int DIST_FOR_SLOW = 400;

	// The maximum power at which the motors should move when the robot is in the slow zone
	const int SLOW_POWER = 40;

	nMotorEncoder[mFrontLeft] = 0;
	nMotorEncoder[mFrontRight] = 0;

	long startTime = nPgmTime;
	int timeToFull = (int) (power / POWER_RATE);

	while (true) {
		int elapsedTime = nPgmTime - startTime;
		int remainingDist = distance - driveEncoderAverage();

		if (remainingDist <= 0) break;

		int currentPower;
		if (elapsedTime < timeToFull) {
			currentPower = (int) (elapsedTime * POWER_RATE);
		} else {
			currentPower = power;
		}
		if (remainingDist <= DIST_FOR_SLOW && currentPower > SLOW_POWER) {
			currentPower = SLOW_POWER;
		}

		currentPower *= direction;
		yolodrive(0, 0, 0, currentPower);

		wait10Msec(1);
	}

	yolodrive(0, 0, 0, 0);
}

/**
 * Wait for enough time to lose momentum
 *
 * This is used in autonomous programs to ensure constistent and precise movements.
 */
void dispenseMomentum()
{
	wait10Msec(75);
}

/*
 * Slide
 */

/**
 * Remove slack from the slide by gentling moving the pulley until resistance
 * from the slide stops the motor
 */
void tenseSlide()
{
	int prevPos = nMotorEncoder[mLiftR];

	motor[mLiftL] = motor[mLiftR] = 10;

	wait10Msec(50);

	while (nMotorEncoder[mLiftR] - prevPos > 10) {
		prevPos = nMotorEncoder[mLiftR];

		wait10Msec(25);
	}

	motor[mLiftL] = motor[mLiftR] = 0;
}

// Lift or lower the slide to the given encoder target
//
// This will automatically use smart power values depending on the direction and
// height of the slide.
//
// The slide will be parked within LIFT_THRESHOLD of the given target.
void liftSlide(int target)
{
	int prevIR = servo[sIR];
	if (prevIR > SERVO_IR_FORWARD) {
		servo[sIR] = SERVO_IR_FORWARD;
		wait10Msec(10);
	}

  while (true) {
    int distance = target - nMotorEncoder[mLiftR];
    if (abs(distance) <= LIFT_THRESHOLD) break;

    int power = 0;
    if (distance > 0) {
      power = 100;
    } else if (distance < 0) {
      if (nMotorEncoder[mLiftR] <= LIFT_SLOW) {
        power = -10;
      } else {
        power = -20;
      }
    }

    motor[mLiftL] = motor[mLiftR] = power;
  }

  motor[mLiftL] = motor[mLiftR] = 0;

  servo[sIR] = prevIR;
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

/*
 * Tongue
 */

// Raise the tongue so balls won't dispense
void raiseTongue()
{
  servo[sTongue] = TONGUE_MAX;
}

// Lower the tongue so balls will dispense
void lowerTongue()
{
  servo[sTongue] = TONGUE_MIN;
}
