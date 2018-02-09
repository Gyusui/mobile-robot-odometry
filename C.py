# -*- coding: utf-8 -*-
"""
Created on Fri Feb  2 14:58:05 2018

@author: 牛帥
"""
# -*- coding: utf-8 -*-
#!/usr/bin/python


import RPi.GPIO as GPIO


GPIO.setwarnings(False)       #warning
GPIO.setmode(GPIO.BOARD)

IN1_R = 8
IN2_R = 10
IN3_L = 16
IN4_L = 18

PWM_R = 12
PWM_L = 22

ROBOT_R = 7.5
WHEEL_R = 4.25
Kp = 0.23
T = 0.05
DC_L = 44
DC_R = 44

count_l = 0
count_r = 0

robot_w = 0.0
robot_v = 0.0




