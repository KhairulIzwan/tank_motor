/*Title: PLL Speed Control*/
/*Author: https://wiki.dfrobot.com/Arduino_Motor_Shield__L298N___SKU_DRI0009_*/
/*Description: PLL Speed Control*/

/*Arduino PWM Speed Control*/
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

void setup()
{
	pinMode(M1, OUTPUT);
	pinMode(M2, OUTPUT);
}

void loop()
{
	int value;
	for(value = 0 ; value <= 255; value+=5)
	{
/*		Direction Control*/
		digitalWrite(M1,HIGH);
		digitalWrite(M2, HIGH);
/*		PLL Speed Control*/
		analogWrite(E1, value);
		analogWrite(E2, value);
		delay(30);
	}
}
