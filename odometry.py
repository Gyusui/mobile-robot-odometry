# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 22:10:26 2018

@author: 牛帥
"""

import robot.py
import time
import math
import M.py

def odometry():
    ipt = input('Ｘ座標とＹ座標を入力してください、コンマを入れてください:')
    data = ipt.split(',')
    data = [int(x) for x in data]
    print('(',data[0],',',data[1],')')
    x_new = 0
    y_new = 0
    robot_th = 0
    while True:
        global robot_w, robot_v
        robot_th_new =robot_th + robot_w*M.T
        x_old = x_new + robot_v*math.cos(robot_th)
        y_old = y_new + robot_v*math.sin(robot_th)
        x_new = x_old
        y_new = y_old
        robot_th = robot_th_new
    if data[0] > 0:
        robot.turnDegreeR(90)
        t_x = abs(x_new)/robot_v
        robot.forward()
        time.sleep(t_x)
        robot.carStop()
        if data[1] > 0:
            robot.turnDegreeL(90)
            t_y = abs(y_new)/robot_v
            robot.forward()
            time.sleep(t_y)
            robot.carStop()
        elif data[1] < 0:
            robot.turnDegreeR(90)
            t_y = abs(y_new)/robot_v
            robot.forward()
            time.sleep(t_y)
            robot.carStop()
        elif data[1] == 0:
           robot.carStop()

    if data[0] < 0:
        robot.turnDegreeL(90)
        t_x = abs(x_new)/robot_v
        robot.forward()
        time.sleep(t_x)
        robot.carStop()
        if data[1]> 0:
            robot.turnDegreeR(90)
            t_y = abs(y_new)/robot_v
            robot.forward()
            time.sleep(t_y)
            robot.carStop()
        elif data[1] < 0:
            robot.turnDegreeL(90)
            t_y = abs(y_new)/robot_v
            robot.forward()
            time.sleep(t_y)
            robot.carStop()
        elif data[1] == 0:
           robot.carStop()

    if data[0] == 0:
        if data[1] > 0:
            t_y = abs(y_new)/robot_v
            robot.forward()
            time.sleep(t_y)
            robot.carStop()
        elif data[1] < 0:
            robot.turnDegreeL(180)
            t_y = abs(y_new)/robot_v
            forward()
            time.sleep(t_y)
            robot.carStop()
        elif data[1] == 0:
            robot.carStop()
if __name__ == '__main__':
    try:

        M.gx
        time.sleep(1)
    except ModuleNotFoundError:
        pass

