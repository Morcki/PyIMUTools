# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 21:36:11 2020

@author: mocki
"""
import numpy as np
import glv
import com
class Config():
    def __init__(self):
        self.imu_data_path = "./data/A15_imu.bin"
        self.gnss_data_path = "./data/gnss.txt"
        
        self.intv = 0.005
        
        self.start_time = 456300
        self.antenna_arm = np.array([0.136, -0.301, -0.184])
        
        self.init_loc = np.array([30.4447873432 * glv.D2R, 114.4718631180 * glv.D2R, 20.918])
        self.init_vel = np.array([-0.001, -0.002, -0.001])
        self.init_pos = np.array([0.85771264, -2.03738244, 184.89962745]) * glv.D2R
        
        self.loc_std = np.array([0.008, 0.009, 0.014])
        self.vel_std = np.array([0.006, 0.006, 0.008])
        self.pos_std = np.array([0.005, 0.005, 0.813]) * glv.D2R
        
        self.GYOARW = 0.03 * glv.D2R / 60    # deg/sqrt(h)
        self.ACCVRW = 0.3 / 60     # m/s/sqrt(h)
        
        # GYO INITIAL BIAS
        self.GYO_BIA_TAU = np.array([4, 4, 4]) * 3600                   # unit : h
        self.GYO_BIA_STD = np.array([0.3, 0.3, 0.3]) * glv.D2R / 3600.0 # unit : deg/h
        self.GYO_BIA_VAL = np.array([0, 0, 0]) * glv.D2R                # unit : deg/s
        # ACC INITIAL BIAS
        self.ACC_BIA_TAU = np.array([4, 4, 4]) * 3600                   # unit : h
        self.ACC_BIA_STD = np.array([30, 30, 30]) * 1e-5                # unit : mGal(1e-3*cm/s^2)
        self.ACC_BIA_VAL = np.array([0, 0, 0]) * 1e-5                   # unit : mGal
        # GYO Scale Factor
        self.GYO_SCF_TAU = np.array([4, 4, 4]) * 3600                   # unit : h
        self.GYO_SCF_STD = np.array([100, 100, 100]) * 1e-6             # unit : ppm
        self.GYO_SCF_VAL = np.array([0, 0, 0]) * 1e-6                   # unit : ppm
        # ACC Scale Factor
        self.ACC_SCF_TAU = np.array([4, 4, 4]) * 3600                   # unit : h
        self.ACC_SCF_STD = np.array([100, 100, 100]) * 1e-6             # unit : ppm
        self.ACC_SCF_VAL = np.array([0, 0, 0]) * 1e-6                   # unit : ppm
        
        init_rn = self.init_loc
        self.init_x = np.hstack((
            init_rn,
            self.init_vel,
            self.init_pos,
            self.GYO_BIA_VAL,
            self.ACC_BIA_VAL,
            self.GYO_SCF_VAL,
            self.ACC_SCF_VAL,
            ))
        
        self.init_Qx = np.diag(np.hstack((
            self.loc_std**2,
            self.vel_std**2,
            self.pos_std**2,
            self.GYO_BIA_STD** 2,
            self.ACC_BIA_STD** 2,
            self.GYO_SCF_STD** 2,
            self.ACC_SCF_STD** 2,
            )))
        