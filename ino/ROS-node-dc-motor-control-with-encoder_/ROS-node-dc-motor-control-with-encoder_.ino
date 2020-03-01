/*Title: The ROS node that controls the DC motor*/
/*Author: https://wiki.dfrobot.com/Arduino_Motor_Shield__L298N___SKU_DRI0009_*/
/*Description: PWM Speed Control*/

/*Re-arrange the code in order to make it works! [Khairul Izwan (24-01-2020)]*/

/*include necessary library*/
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//Arduino PWM Speed Control
//1. Left
//2. Right
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;
 
//ROTARY ENCODER PINS (CHANGE ACCORDINGLY)
#define ENCODER_COUNT_RIGHT_UP 18
#define ENCODER_COUNT_RIGHT_DOWN 19
#define ENCODER_COUNT_LEFT_UP 3
#define ENCODER_COUNT_LEFT_DOWN 2

//This variable will increase or decrease depending on the 
//rotation of encoder
volatile signed int TEMP_LEFT, COUNTER_LEFT = 0; 
volatile signed int TEMP_RIGHT, COUNTER_RIGHT = 0;

#define wheelSep 0.14 // mm
#define wheelRadius 0.05; // mm

float transVelocity;
float rotVelocity;
float velDiff;

float leftPWM;
float rightPWM;

float leftPower;
float rightPower;

char transVel[10];
char rotVel[10];

// minimum speed for the motor
int min_speed=150;

// helper function for cmd_vel topic
void messageCb(const geometry_msgs::Twist &msg)
{
	transVelocity = msg.linear.x;
	rotVelocity = msg.angular.z;

	velDiff = (wheelSep * rotVelocity) / 2.0;
	leftPower = (transVelocity + velDiff) / wheelRadius;
	rightPower = (transVelocity - velDiff) / wheelRadius;

//	Convert to PWM DC (0 ~ 100)
	leftPWM = (leftPower - 0) * (255 - 0) / (4.4 - 0) + 0;
	rightPWM = (rightPower - 0) * (255 - 0) / (4.4 - 0) + 0;

	motorDirection();
}

void motorDirection()
{
	if (leftPWM > 0 and rightPWM > 0)
	{
//		tank forward
		digitalWrite(M1, HIGH);
		digitalWrite(M2, HIGH);
		analogWrite(E1, max(abs(leftPWM), min_speed));
		analogWrite(E2, max(abs(rightPWM), min_speed));
	}
	else if (leftPWM < 0 and rightPWM < 0)
	{
//		tank backward
		digitalWrite(M1, LOW);
		digitalWrite(M2, LOW);
		analogWrite(E1, max(abs(leftPWM), min_speed));
    analogWrite(E2, max(abs(rightPWM), min_speed));
	}
	else if (leftPWM < 0 and rightPWM > 0)
	{
//		tank turn left
		digitalWrite(M1, LOW);
		digitalWrite(M2, HIGH);
    analogWrite(E1, max(abs(leftPWM), min_speed));
    analogWrite(E2, max(abs(rightPWM), min_speed));
	}
	else if (leftPWM > 0 and rightPWM < 0)
	{
//    tank turn right
		digitalWrite(M1, HIGH);
		digitalWrite(M2, LOW);
    analogWrite(E1, max(abs(leftPWM), min_speed));
    analogWrite(E2, max(abs(rightPWM), min_speed));
	}
	else if (leftPWM == 0 and rightPWM == 0)
	{
//    tank stalled
		analogWrite(E1, 0);
		analogWrite(E2, 0);
	}
}

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 encLeft;
std_msgs::Float32 encRight;
ros::Publisher pub_encLeft("left_encoder", &encLeft);
ros::Publisher pub_encRight("right_encoder", &encRight);

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb);
ros::Subscriber<std_msgs::Bool> sub_reset_leftEnc("/reset_leftEnc", messageCb_reset_leftEnc);
ros::Subscriber<std_msgs::Bool> sub_reset_rightEnc("/reset_rightEnc", messageCb_reset_rightEnc);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
//	SETTING UP PINMODE
//	1. PWM
//	2. DIRECTION
	pinMode(ENCODER_COUNT_RIGHT_UP, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_RIGHT_DOWN, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_LEFT_UP, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_LEFT_DOWN, INPUT_PULLUP);

//  SETTING UP INTERRUPT
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_LEFT_UP), COUNT_INTERRUPT_LEFT_CW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_LEFT_DOWN), COUNT_INTERRUPT_LEFT_CCW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_RIGHT_UP), COUNT_INTERRUPT_RIGHT_CW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_RIGHT_DOWN), COUNT_INTERRUPT_RIGHT_CCW, RISING);

	pinMode(M1, OUTPUT);
	pinMode(M2, OUTPUT);

	nh.initNode();
	
	nh.advertise(pub_encLeft);
	nh.advertise(pub_encRight);
	
	nh.subscribe(sub);
  
	nh.subscribe(sub_reset_leftEnc);
  nh.subscribe(sub_reset_rightEnc);
}

//put your main code here, to run repeatedly:
void loop()
{
	encLeft.data = COUNTER_LEFT;
	encRight.data = COUNTER_RIGHT;

	pub_encLeft.publish(&encLeft);
	pub_encRight.publish(&encRight);
  
	nh.spinOnce();
	delay(10);
}

/*
* ENCODER COUNTER FUNCTION
*/
void COUNT_INTERRUPT_LEFT_CW() 
{
	if(digitalRead(ENCODER_COUNT_LEFT_DOWN)==LOW) 
	{
		COUNTER_LEFT++;
	}
	else
	{
		COUNTER_LEFT--;
	}
}

void COUNT_INTERRUPT_LEFT_CCW()
{
	if(digitalRead(ENCODER_COUNT_LEFT_UP)==LOW) 
	{
		COUNTER_LEFT--;
	}
	else
	{
		COUNTER_LEFT++;
	}
}

void COUNT_INTERRUPT_RIGHT_CW() 
{
	if(digitalRead(ENCODER_COUNT_RIGHT_DOWN)==LOW) 
	{
		COUNTER_RIGHT++;
	}
	else
	{
		COUNTER_RIGHT--;
	}
}

void COUNT_INTERRUPT_RIGHT_CCW() 
{
	if(digitalRead(ENCODER_COUNT_RIGHT_UP)==LOW) 
	{
		COUNTER_RIGHT--;
	}
	else
	{
		COUNTER_RIGHT++;
	}
}

void messageCb_reset_leftEnc(const std_msgs::Bool &msg)
{
	if (msg.data == true)
	{
		COUNTER_LEFT = 0;
	}
}

void messageCb_reset_rightEnc(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_RIGHT = 0;
  }
}
