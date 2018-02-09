# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 15:13:19 2018

@author: 牛帥
"""


import RPi.GPIO as GPIO
import math
import threading
import time
import queue

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



def init():
    GPIO.setup(IN1_R, GPIO.OUT)
    GPIO.setup(IN2_R, GPIO.OUT)
    GPIO.setup(PWM_R, GPIO.OUT)  #right motor init
    GPIO.setup(IN3_L, GPIO.OUT)
    GPIO.setup(IN4_L, GPIO.OUT)
    GPIO.setup(PWM_L, GPIO.OUT)  #left motor

def right_wheel_forward(dc_r, t):
    pwmR.ChangeDutyCycle(dc_r)
    GPIO.output(IN1_R, GPIO.HIGH)
    GPIO.output(IN2_R, GPIO.LOW)
    time.sleep(t)


def right_wheel_backward(dc_r, t):
    pwmR.ChangeDutyCycle(dc_r)
    GPIO.output(IN1_R, GPIO.LOW)
    GPIO.output(IN2_R, GPIO.HIGH)
    time.sleep(t)


def left_wheel_forward(dc_l, t):
    pwmL.ChangeDutyCycle(dc_l)
    GPIO.output(IN3_L, GPIO.HIGH)
    GPIO.output(IN4_L, GPIO.LOW)
    time.sleep(t)


def left_wheel_backward(dc_r, t):
    pwmL.ChangeDutyCycle(dc_l)
    GPIO.output(IN3_L, GPIO.LOW)
    GPIO.output(IN4_L, GPIO.HIGH)
    time.sleep(t)


def R_forward(dc_l, dc_r, t):
    left_wheel_forward(dc_l, t)     #車輪の違う時間がほしい、どうすればいいのか
    right_wheel_forward(dc_r, t)
    time.sleep(t)


def get_robot_v(wheel_vl, wheel_vr):
    return (wheel_vl + wheel_vr)/2


def get_robot_w(wheel_vl, wheel_vr):
    return (wheel_vr - wheel_vl)/(2*ROBOT_R)


def R_stop(t):
    pwmL.ChangeDutyCycle(0)
    pwmR.ChangeDutyCycle(0)
    time.sleep(t)


def leftWheel(channel):    #左車輪は割り込みを基づいて、カウンタ数を記録する
    global COUNT_L         #カウンタ割り込み
    if channel == 13:
        COUNT_L += 1


def rightWheel(channel):  #右車輪は割り込みを基づいて、カウンタ数を記録する
    global COUNT_R        #カウンタ割り込み
    if channel == 11:
        COUNT_R += 1


def leftReader(q,T):       #左車輪カウンタリーダータイマー割り込みとして
    while True:            #一定時間間隔内に、カウンタ数を読む。その結果を車輪の角速度を計算する
        try:
            q.get(block=False)
        except queue.Empty:
            pass
        global COUNT_L
        W_L = ((2*(math.pi)/20)*COUNT_L)/T
        q.put(W_L)
        COUNT_L = 0           #一定時間内の角速度の計算が完成したら、カウンタ数をリセットする
        time.sleep(T)         #ここはちょっと問題がある、おいときます！


def rightReader(q,T):     #右車輪カウンタリーダー,タイマー割り込みとして
    while True:           #一定時間間隔内に、カウンタ数を読む。その結果を車輪の角速度を計算する
        try:
            q.get(block=False)
        except queue.Empty:
            pass
        global COUNT_R
        W_R = ((2*(math.pi)/20)*COUNT_R)/T
        q.put(W_R)
        COUNT_R = 0
        time.sleep(T)




def degree_radian(degree):
    return degree * (math.pi)/180.0

def radian_degree(radian):
    return radian * (180.0/math.pi)

'''
特定な角度を回転するために、現在の角度と目標角度を引数として、回転を実現する
'''
def turnLeft(goal_angle, current_angle):
     goal_radian = degree_radian(goal_angle)
     current_radian = degree_radian(current_angle)

     while goal_radian > current_radian:
        robot_w_target = (Kp*(robotDeg_target - robotDeg_now))/T
        robot_v_target = robot_w_target*robot_r
        wheel_vr_target = wheel_vl_target = robot_v_target
        wheel_wl = q_1.get()
        wheel_wr = q_2.get()
        wheel_vl = wheel_wl*wheel_r
        wheel_vr = wheel_wr*wheel_r
        global DC_L, DC_R
        duty1 = Kp*(wheel_vl_target -wheel_vl) + DC_L
        if duty1 > 100:
            duty1 = 99
        elif duty1 < 0:
            duty1 = 0
        DC_L = duty1
        duty2 = Kp*(wheel_vr_target -wheel_vr) + DC_R
        if duty2 > 100:
            duty2 = 99
        elif duty2 < 0:
            duty2 = 0
            print("0")
        DC_R = duty2
        robot_w = (wheel_vr - (-wheel_vl))/(2*robot_r)
        robot_v = (wheel_vl + wheel_vr)/2
        robotDeg_now = robotDeg_now +robot_w*t
    if robotDeg_now >= robotDeg_target:
        break


def turnDegreeR(degree):
    global robotDeg_now
    robotDeg_now = (robotDeg_now*(math.pi))/180
    robotDeg_target =(degree*(math.pi))/180
    while True:
        if robotDeg_target > robotDeg_now:
            turnRight()
            robot_w_target = (Kp*(robotDeg_target - robotDeg_now))/t
            robot_v_target = robot_w_target*robot_r
            wheel_vr_target = wheel_vl_target = robot_v_target
            wheel_wl = q_1.get()
            wheel_wr = q_2.get()
            wheel_vl = wheel_wl*wheel_r
            wheel_vr = wheel_wr*wheel_r
            global DC_L, DC_R
            duty1 = Kp*(wheel_vl_target -wheel_vl) + DC_L
            if duty1 > 100:
                duty1 = 99
            elif duty1 < 0:
                duty1 = 0
            DC_L = duty1
            duty2 = Kp*(wheel_vr_target -wheel_vr) + DC_R
            if duty2 > 100:
                duty2 = 99
            elif duty2 < 0:
                duty2 = 0
                print("0")
            DC_R = duty2
            robot_w = (wheel_vr - (-wheel_vl))/(2*robot_r)
            robotDeg_now = robotDeg_now +robot_w*t
            #print("robotDeg_now:%s" %robotDeg_now)
            #print("robotDeg_target:%s"%robotDeg_target)
        if robotDeg_now >= robotDeg_target:
            break

GPIO.setup(13, GPIO.IN)
GPIO.setup(11, GPIO.IN)
GPIO.add_event_detect(13, GPIO.RISING, callback=leftWheel, bouncetime=1)
GPIO.add_event_detect(11, GPIO.RISING, callback=rightWheel, bouncetime=1)


if __name__ == '__main__':    #メイン関数
    robotDeg_now = 0.0
    init()                             #初期化。ピンとPWMの初期化
    pwmR = GPIO.PWM(PWM_R, 50)
    pwmL = GPIO.PWM(PWM_L, 50)
    pwmR.start(0)
    pwmL.start(0)
    q_1 = queue.Queue(maxsize=1)      #queueの初期化、sizeは1です
    q_2 = queue.Queue(maxsize=1)

    thread_1 = threading.Thread(None, leftReader, kwargs={'q': q_1,'t':t})
    thread_2 = threading.Thread(None, rightReader, kwargs={'q': q_2,'t':t})

    thread_1.start()
    thread_2.start()
    try:
        while True:
            turnLeft()
#            turnDegree(90)
#            carStop()
#            time.sleep(1)
    except KeyboardInterrupt:
        pass

GPIO.remove_event_detect(13)
GPIO.remove_event_detect(11)
GPIO.cleanup()