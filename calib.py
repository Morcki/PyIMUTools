# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 12:52:14 2020

@author: Mocki

E-mail : 605095234@qq.com

TO : Art is piece of luxury
"""
import numpy as np
from numpy.linalg import inv
from glv import *
# Calibration of Accelerometer and Gyroscope 
class Calibration():
    def __init__(self,imu_data):
        self.imu_data = imu_data
    def input_cuttime(self,ts,te,dtype):
        "dtype : ACC / GYO"
        if dtype == "ACC":
            return self.imu_data.imu_avg(ts,te,dtype)
        elif dtype == "GYO":
            return self.imu_data.imu_sum(ts,te,dtype)
    def acc_calib(self,*arg):
        '''
        arg : fx_ fx fy_ fy fz_ fz
        note:
            fx_ -> x direction down
            fx  -> x direction up
        '''
        # build up matrix A
        A = self.imu_data.imu_std()
        # build up matrix L
        L = np.zeros((3,6))
        for i,f in enumerate(arg):
            L[:,i] = f
        M = L @ A.T @ inv(A @ A.T)
        return M
    def gyo_calib(self,dt,l,l_,alpha):
        '''
        note:
            l     -> rotation angle of x/y/z axis (clockwise)
            l_    -> rotation angle of x/y/z axis (counter-clockwise)
            alpha -> rotation angle
        '''
        bg = (l + l_) / (2 * dt) - EARTH_ROTATE * np.sin(self.imu_data.fi)
        deta_s = (l - l_) / (2 * alpha) - 1
        return bg, deta_s
    