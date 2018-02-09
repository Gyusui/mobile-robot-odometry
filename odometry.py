# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 22:10:26 2018

@author: 牛帥
"""

import robot.py
import matplotlib.pyplot as plt
import time
import M.py




if __name__ == '__main__':
    try:

        M.gx
        time.sleep(1)
    except ModuleNotFoundError:
        pass

