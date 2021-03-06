//Title: The ROS node that controls the DC motor
//Author: https://wiki.dfrobot.com/Arduino_Motor_Shield__L298N___SKU_DRI0009_
//Description: PWM Speed Control

//Re-arrange the code in order to make it works! [Khairul Izwan (24-01-2020)]

//include necessary library
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/RegionOfInterest.h>

ros::NodeHandle nh;

//Arduino PWM Speed Control
//1. Left
//2. Right
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

//Direction Control
//Backward
//digitalWrite(M1,HIGH);
//digitalWrite(M2, HIGH);
//Forward
//digitalWrite(M1,LOW);
//digitalWrite(M2, LOW);
//Left
//digitalWrite(M1,HIGH);
//digitalWrite(M2, LOW);
//Right
//digitalWrite(M1,LOW);
//digitalWrite(M2, HIGH);

#define wheelSep 0.14 // mm
#define wheelRadius 0.05; // mm

float transVelocity;
float rotVelocity;
float velDiff;

float leftPWM;
float rightPWM;

float leftPower;
float rightPower;

int img_width;
int img_height;

char transVel[10];
char rotVel[10];

void vel_messageCb(const geometry_msgs::Twist &msg)
{
//	convert float to char
//	dtostrf(msg.linear.x, 0, 2, transVel);
//	dtostrf(msg.angular.z, 0, 2, rotVel);

//	nh.loginfo("Linear Velocity:\t");
//	nh.loginfo(transVel);
//	nh.loginfo("Angular Velocity:\t");
//	nh.loginfo(rotVel);

	transVelocity = msg.linear.x;
	rotVelocity = msg.angular.z;

	velDiff = (wheelSep * rotVelocity) / 2.0;
	leftPower = (transVelocity + velDiff) / wheelRadius;
	rightPower = (transVelocity - velDiff) / wheelRadius;

//	convert float to char
//	dtostrf(leftPWM, 0, 2, transVel);
//	dtostrf(rightPWM, 0, 2, rotVel);

//	nh.loginfo("Linear Velocity:\t");
//	nh.loginfo(transVel);
//	nh.loginfo("Angular Velocity:\t");
//	nh.loginfo(rotVel);

//	Convert to PWM DC (0 ~ 100)
	leftPWM = (leftPower - 0) * (255 - 0) / (4.4 - 0) + 0;
	rightPWM = (rightPower - 0) * (255 - 0) / (4.4 - 0) + 0;

//	convert float to char
//	dtostrf(leftPWM, 0, 2, transVel);
//	dtostrf(rightPWM, 0, 2, rotVel);

//	nh.loginfo("Linear Velocity:\t");
//	nh.loginfo(transVel);
//	nh.loginfo("Angular Velocity:\t");
//	nh.loginfo(rotVel);

	motorDirection();
}

void motorDirection()
{
	if (leftPWM > 0 and rightPWM > 0)
	{
//		tank forward
//		nh.loginfo("[WARN] Forward...");
		digitalWrite(M1, LOW);
		digitalWrite(M2, LOW);
		analogWrite(E1, max(abs(leftPWM), 200));
		analogWrite(E2, max(abs(rightPWM), 200));
	}
	else if (leftPWM < 0 and rightPWM < 0)
	{
//		tank backward
//		nh.loginfo("[WARN] Backward...");
		digitalWrite(M1, HIGH);
		digitalWrite(M2, HIGH);
		analogWrite(E1, max(abs(leftPWM), 200));
		analogWrite(E2, max(abs(rightPWM), 200));
	}
	else if (leftPWM < 0 and rightPWM > 0)
	{
//		tank turn left
//		nh.loginfo("[WARN] Left...");
		digitalWrite(M1, LOW);
		digitalWrite(M2, HIGH);
		analogWrite(E1, max(abs(leftPWM), 200));
		analogWrite(E2, max(abs(rightPWM), 200));
	}
	else if (leftPWM > 0 and rightPWM < 0)
	{
//		tank turn right
//		nh.loginfo("[WARN] Right...");
		digitalWrite(M1, HIGH);
		digitalWrite(M2, LOW);
		analogWrite(E1, max(abs(leftPWM), 200));
		analogWrite(E2, max(abs(rightPWM), 200));
	}
	else if (leftPWM == 0 and rightPWM == 0)
	{
//		tank stalled
//		nh.loginfo("[INFO] Stop!...");
		analogWrite(E1, 0);
		analogWrite(E2, 0);
	}
}

void roi_messageCb(const sensor_msgs::RegionOfInterest &msg)
{
	nh.loginfo("ROI found!");

//	convert float to char
//	dtostrf(msg.x_offset, 0, 2, transVel);
//	dtostrf(msg.y_offset, 0, 2, rotVel);

//	nh.loginfo("X:\t");
//	nh.loginfo(transVel);
//	nh.loginfo("Y:\t");
//	nh.loginfo(rotVel);
}

void img_messageCb(const sensor_msgs::CameraInfo &msg)
{
  nh.loginfo("CameraInfo found!");
  
//  convert float to char
//  dtostrf(msg.height, 0, 2, transVel);
//  dtostrf(msg.width, 0, 2, rotVel);
//
//  nh.loginfo("Height:\t");
//  nh.loginfo(transVel);
//  nh.loginfo("Width:\t");
//  nh.loginfo(rotVel);
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", vel_messageCb);
ros::Subscriber<sensor_msgs::RegionOfInterest> sub_roi("/roi", roi_messageCb);
ros::Subscriber<sensor_msgs::CameraInfo> sub_img("/camPi/camera_info", img_messageCb);

//put your setup code here, to run once:
void setup()
{
	pinMode(M1, OUTPUT);
	pinMode(M2, OUTPUT);

	nh.initNode();
//	nh.subscribe(sub_cmd_vel);
	nh.subscribe(sub_roi);
  nh.subscribe(sub_img);
}

//put your main code here, to run repeatedly:
void loop()
{
	nh.spinOnce();
	delay(1);
}
