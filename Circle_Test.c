
task main()
{
	a = 0;
	while(a < 500)
	{
		motor[RightFront] = 100;
		motor[RightBack] = 100;
		motor[LeftFront] = 10;
		motor[LeftBack]= 10;
		a++;
	}
}
