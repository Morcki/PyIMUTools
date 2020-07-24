# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 12:52:14 2020

@author: Mocki

E-mail : 605095234@qq.com

TO : Art is piece of luxury
"""
import matplotlib.pyplot as plt
from glv import *
class ChkData():
    def __init__(self,imu_data):
        self.imu_chk = imu_data
        self.intv = imu_data.intv
    def plot_timef(self,dtype):
        fig = plt.figure(figsize=(10,6))
        ax  = fig.add_subplot(111)
        if dtype == "ACC":
            X = self.imu_chk.imu_mat("TIM")
            Y = self.imu_chk.imu_mat("ACC")
            ax.plot(X,Y[:,0],label="ax")
            ax.plot(X,Y[:,1],label="ay")
            ax.plot(X,Y[:,2],label="az")
            ax.legend(loc="best")
        elif dtype == "GYO":
            X = self.imu_chk.imu_mat("TIM")
            Y = self.imu_chk.imu_mat("GYO")
            ax.plot(X,Y[:,0],label="gx")
            ax.plot(X,Y[:,1],label="gy")
            ax.plot(X,Y[:,2],label="gz")
            ax.legend(loc="best")
        elif dtype == "ALL":
            X = self.imu_chk.imu_mat("TIM")
            Y = self.imu_chk.imu_mat("ALL")[:,1:]
            ax.plot(X,Y[:,0],label="gx")
            ax.plot(X,Y[:,1],label="gy")
            ax.plot(X,Y[:,2],label="gz")
            ax.plot(X,Y[:,3],label="ax")
            ax.plot(X,Y[:,4],label="ay")
            ax.plot(X,Y[:,5],label="az")
            ax.legend(loc="best")
        else:
            raise Exception("Wrong dtype[%s]." % dtype)
    def plot_epoch(self,dtype):
        fig = plt.figure(figsize=(10,6))
        ax  = fig.add_subplot(111)
        if dtype == "ACC":
            X = self.imu_chk.imu_mat("TIM");X -= X[0]
            Y = self.imu_chk.imu_mat("ACC")
            ax.plot(X,Y[:,0],label="ax")
            ax.plot(X,Y[:,1],label="ay")
            ax.plot(X,Y[:,2],label="az")
            ax.legend(loc="best")
        elif dtype == "GYO":
            X = self.imu_chk.imu_mat("TIM");X -= X[0]
            Y = self.imu_chk.imu_mat("GYO")
            ax.plot(X,Y[:,0],label="gx")
            ax.plot(X,Y[:,1],label="gy")
            ax.plot(X,Y[:,2],label="gz")
            ax.legend(loc="best")
        elif dtype == "ALL":
            X = self.imu_chk.imu_mat("TIM");X -= X[0]
            Y = self.imu_chk.imu_mat("ALL")[:,1:]
            ax.plot(X,Y[:,0],label="gx")
            ax.plot(X,Y[:,1],label="gy")
            ax.plot(X,Y[:,2],label="gz")
            ax.plot(X,Y[:,3],label="ax")
            ax.plot(X,Y[:,4],label="ay")
            ax.plot(X,Y[:,5],label="az")
            ax.legend(loc="best")
        else:
            raise Exception("Wrong dtype[%s]." % dtype)
class Chkalg():
    def __init__(self,alg_mat):
        self.alg_mat = alg_mat
    def plot(self,ptype):
        fig = plt.figure(figsize=(10,6))
        if ptype == "A":
            ax  = fig.add_subplot(111)    
            X = self.alg_mat[:,0]
            Y = self.alg_mat[:,1] * R2D
        elif ptype == "B":
            ax  = fig.add_subplot(111)    
            X = self.alg_mat[:,0]
            Y = self.alg_mat[:,2] * R2D
        elif ptype == "F":
            ax  = fig.add_subplot(111)    
            X = self.alg_mat[:,0]
            Y = self.alg_mat[:,3] * R2D
        elif ptype == "ALL":
            ax  = fig.add_subplot(111)    
            X = self.alg_mat[:,0]
            Y = self.alg_mat[:,1:] * R2D
            ax.plot(X,Y[:,0],label="alpha")
            ax.plot(X,Y[:,1],label="theta")
            ax.plot(X,Y[:,2],label="fi")
            ax.legend(loc="best")
            