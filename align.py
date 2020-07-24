# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 12:52:14 2020

@author: Mocki

E-mail : 605095234@qq.com

TO : Art is piece of luxury
"""

# alignment of Accelerometer and Gyroscope 
import numpy as np
from numpy.linalg import inv
class Align():
    def __init__(self, imu_data):
        self.imu_data = imu_data
        # alignment data
        self.alg_time  = []
        self.alg_alpha = []
        self.alg_theta = []
        self.alg_fi    = []
    def __clear(self):
        self.alg_time  = []
        self.alg_alpha = []
        self.alg_theta = []
        self.alg_fi    = []
    def alg_mat(self,mat_type):
        if mat_type == "T":
            return np.array(self.alg_time)
        if mat_type == "A":
            return np.array(self.alg_alpha)
        elif mat_type == "B":
            return np.array(self.alg_theta)
        elif mat_type == "F":
            return np.array(self.alg_fi)
        elif mat_type == "ALL":
            if len(self.alg_time) == 1:
                return np.array(self.alg_time + self.alg_alpha + self.alg_theta + self.alg_fi)
            else:
                return np.array([[p] + [i] + [j] + [k] for p,i,j,k in zip(self.alg_time,self.alg_alpha,self.alg_theta,self.alg_fi)])
        else:
            raise Exception("Wrong MAT TYPE[%s]" % mat_type)
    def align_mode(self, mode):
        self.__clear()
        N = self.imu_data.imu_alg()
        if mode == 0: # average all the data
            ts = self.imu_data.imu_tim[0]
            te = self.imu_data.imu_tim[-1]
            gama_b,_ = self.imu_data.imu_avg(ts,te,"ACC")
            omega_b,_ = self.imu_data.imu_avg(ts,te,"GYO")
            B = self.__build_B(gama_b,omega_b)
            Cn2b = B @ inv(N)
            a,b,f = self.__cop_atu(Cn2b)
            self.__load_atu(0,a,b,f)
            return 1
            
        elif mode == -1: # epoch by epoch
            intv = self.imu_data.intv
            for idata in self.imu_data:
                gama_b = np.array(idata[4:]) / intv
                omega_b = np.array(idata[1:4]) / intv
                B = self.__build_B(gama_b,omega_b)
                Cn2b = B @ inv(N)
                a,b,f = self.__cop_atu(Cn2b)
                self.__load_atu(idata[0],a,b,f)
            return 1
        
        elif mode > 0:
            t0 = self.imu_data.imu_tim[0]
            t = t0
            while 1:
                ts = t
                te = t + mode
                gama_b,_ = self.imu_data.imu_avg(ts,te,"ACC")
                omega_b,_ = self.imu_data.imu_avg(ts,te,"GYO")
                B = self.__build_B(gama_b,omega_b)
                Cn2b = B @ inv(N)
                a,b,f = self.__cop_atu(Cn2b)
                self.__load_atu((ts+te)/2,a,b,f)
                t += mode
                if t > self.imu_data.imu_tim[-1]:
                    break
            return 1
        else:
            return 0
        
        
    def __build_B(self,gama_b,omega_b):
        v_b = np.cross(gama_b,omega_b)
        B = np.zeros((3,3))
        B[:,0] = gama_b;B[:,1] = omega_b;B[:,2] = v_b
        return B
    
    def __cop_atu(self,Cn2b):
        "compute attitude angle"
        alpha = np.arctan(Cn2b[1,0]/Cn2b[0,0])
        theta = np.arctan(-Cn2b[2,0]/np.sqrt(Cn2b[2,1]**2 + Cn2b[2,2]**2))
        fi    = np.arctan(Cn2b[2,1]/Cn2b[2,2])
        return alpha,theta,fi
    
    def __load_atu(self,t,a,b,f):
        self.alg_time.append(t)
        self.alg_alpha.append(a)
        self.alg_theta.append(b)
        self.alg_fi.append(f)