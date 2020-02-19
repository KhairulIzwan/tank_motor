#!/usr/bin/env python

#Title: Python Subscriber for Tank Navigation
#Author: Khairul Izwan Bin Kamsani - [23-01-2020]
#Description: Tank Navigation Subcriber Nodes (Python)

#remove or add the library/libraries for ROS
import rospy
import time
import RPi.GPIO as GPIO
 
#remove or add the message type
from geometry_msgs.msg import Twist

"""
To import the module and check to see if it is successful:
try:
	import RPi.GPIO as GPIO
except RuntimeError:
	print("Error importing RPi.GPIO!  This is probably because you need superuser privileges. You can achieve this by using 'sudo' to run your script")
"""
import RPi.GPIO as GPIO

"""
Pin numbering

There are two ways of numbering the IO pins on a Raspberry Pi within RPi.GPIO. The first is using the BOARD numbering system. This refers to the pin numbers on the P1 header of the Raspberry Pi board. The advantage of using this numbering system is that your hardware will always work, regardless of the board revision of the RPi. You will not need to rewire your connector or change your code.

The second numbering system is the BCM numbers. This is a lower level way of working - it refers to the channel numbers on the Broadcom SOC. You have to always work with a diagram of which channel number goes to which pin on the RPi board. Your script could break between revisions of Raspberry Pi boards.
"""

#Declare the GPIO settings
GPIO.setmode(GPIO.BCM)

"""
Warnings

It is possible that you have more than one script/circuit on the GPIO of your Raspberry Pi. As a result of this, if RPi.GPIO detects that a pin has been configured to something other than the default (input), you get a warning when you try to configure a script. To disable these warnings:

GPIO.setwarnings(False)
"""

#Warnings
GPIO.setwarnings(False)

"""
Setup a channel

Input
GPIO.setup(channel, GPIO.IN)

Output
GPIO.setup(channel, GPIO.OUT)
"""

#Set up GPIO pins
pwmPin_Right = 13
dirPin_Right = 24
pwmPin_Left = 12
dirPin_Left = 26

GPIO.setup(pwmPin_Right, GPIO.OUT)
GPIO.setup(dirPin_Right, GPIO.OUT)
GPIO.setup(pwmPin_Left, GPIO.OUT)
GPIO.setup(dirPin_Left, GPIO.OUT)

"""
Using PWM in RPi.GPIO

To create a PWM instance:

p = GPIO.PWM(channel, frequency)
To start PWM:

p.start(dc)   # where dc is the duty cycle (0.0 <= dc <= 100.0)
To change the frequency:

p.ChangeFrequency(freq)   # where freq is the new frequency in Hz
To change the duty cycle:

p.ChangeDutyCycle(dc)  # where 0.0 <= dc <= 100.0
To stop PWM:

p.stop()
Note that PWM will also stop if the instance variable 'p' goes out of scope.
"""

#Set the motor speed
p_Right = GPIO.PWM(pwmPin_Right, 100)
p_Left = GPIO.PWM(pwmPin_Left, 100)

#Tank parameters
wheelSep = 0.14
wheelRadius = 0.05

#define function/functions to provide the required functionality
def speed_callback(msg):
#	Print the actual data recieved
#	print("Linear Vel %s \tRotation Vel %s " % (msg.linear.x, msg.angular.z))
	
	velDiff = (wheelSep * msg.angular.z) / 2.0
	leftPower = (msg.linear.x + velDiff) / wheelRadius
	rightPower = (msg.linear.x - velDiff) / wheelRadius
	 
#	print("leftPower %s \trightPower %s " % (leftPower, rightPower))

#	Convert to PWM DC (0 ~ 100)
	leftPWM = (leftPower - 0) * (100 - 0) / (4.4 - 0) + 0
	rightPWM = (rightPower - 0) * (100 - 0) / (4.4 - 0) + 0
	
	print("leftPower %s \tleftPWM %s " % (leftPower, leftPWM))
	print("rightPower %s \trightPWM %s " % (rightPower, rightPWM))
	
	tank_navi(leftPWM, rightPWM)
	
def tank_navi(left, right):
	if left > 0 and right > 0:
#		tank forward
		p_Right.start(abs(left))
		p_Left.start(abs(right))
		GPIO.output(dirPin_Right, GPIO.HIGH)
		GPIO.output(dirPin_Left, GPIO.LOW)
	elif left < 0 and right < 0:
#		tank backward
		p_Right.start(abs(left))
		p_Left.start(abs(right))
		GPIO.output(dirPin_Right, GPIO.LOW)
		GPIO.output(dirPin_Left, GPIO.HIGH)
	elif left < 0 and right > 0:
#		tank turn left
		p_Right.start(abs(left))
		p_Left.start(abs(right))
		GPIO.output(dirPin_Right, GPIO.HIGH)
		GPIO.output(dirPin_Left, GPIO.HIGH)
	elif left > 0 and right < 0:
#		tank turn right
		p_Right.start(abs(left))
		p_Left.start(abs(right))
		GPIO.output(dirPin_Right, GPIO.LOW)
		GPIO.output(dirPin_Left, GPIO.LOW)

if __name__=='__main__':
	#Add here the name of the ROS. In ROS, names are unique named.
	rospy.init_node('tank_vel')

	#subscribe to a topic using rospy.Subscriber class
	sub=rospy.Subscriber('/cmd_vel', Twist, speed_callback)
	rospy.spin()
