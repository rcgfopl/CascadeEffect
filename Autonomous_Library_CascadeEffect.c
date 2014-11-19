const int DRIVE_DIST=0;
const int MAX_LIFT_HEIGHT=1000;

void First_Autonomous()
{
	Forward(DRIVE_DIST, 100);
	Turn(450,100,Right);
	while(SensorValue(IR) != 5)
	{
		moveCircle(100, 100);
	}
	Turn(450,100,Left);
	while(nMotorEncoder(mLift1)<900)
	{
		if(nMotorEncoder(mLift1) < MAX_LIFT_HEIGHT)
		{
			motor[mLift1] = 100;
			motor[mLift2] = 100;
		}
	}
	Forward(200,70);
	//motor[mTrapdoor] = 50;
	wait1Msec(500);
	Backward(200,70);
	while(nMotorEncoder(mLift1)>0)
	{
		motor[mLift1] = 100;
		motor[mLift2] = 100;
	}
	Turn(450,100,right);
}
