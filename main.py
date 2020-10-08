# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 12:52:14 2020

@author: Mocki

E-mail : 605095234@qq.com

TO : Art is piece of luxury
"""
import struct
import chkimu
import imu
import calib
import align
import glv
import config as conf
import numpy as np
import matplotlib.pyplot as plt
## check raw data

#ts = 0
#inloc = [0, 30.4042231878 * D2R, 0]
#invel = [0,0,0]
#inpos = [0,0,0]
# imu_data = imu.ImuData(0.005)
# imu_data.load_initstatus(ts, inloc, invel, inpos)
# imu_data.loadf("./data/Static_Data.txt")# Calibration_Data Static_Data
# imu_data.loadbinf("./data/Data1.bin")
# chker = chkimu.ChkData(imu_data)
# chker.plot_epoch("ACC")
# chker.plot_epoch("GYO")


################################################
## Calibration of Accelerometer and Gyroscope ##
################################################

#t0 = imu_data.imu_tim[0]
#intv = imu_data.intv
#caber = calib.Calibration(imu_data)
#fz_,_ = caber.input_cuttime(t0+394.75,t0+398.2,"ACC")
#fz,_  = caber.input_cuttime(t0+510.7,t0+923.4,"ACC")
#fx,_  = caber.input_cuttime(t0+931.6,t0+1216.9,"ACC")
#fx_,_ = caber.input_cuttime(t0+1219.8,t0+1474,"ACC")
#fy_,_ = caber.input_cuttime(t0+1481,t0+1872,"ACC")
#fy,_  = caber.input_cuttime(t0+1879.5,t0+2125.5,"ACC")
#M = caber.acc_calib(fx_,fx,fy_,fy,fz_,fz)
#print("ACC CALIBRATION RESULTS:\n",M)
#lz_,n1 = caber.input_cuttime(t0+435.43,t0+445.43,"GYO")
#lz,n2  = caber.input_cuttime(t0+469.86,t0+479.86,"GYO")
#bgz,beta_sz = caber.gyo_calib((n1+n2)/2*intv,lz[2],lz_[2],90 * D2R)
#lx_,n1 = caber.input_cuttime(t0+1194.51,t0+1204.51,"GYO")
#lx,n2  = caber.input_cuttime(t0+1161.25,t0+1171.25,"GYO")
#bgx,beta_sx = caber.gyo_calib((n1+n2)/2*intv,lx[0],lx_[0],90 * D2R)
#ly_,n1  = caber.input_cuttime(t0+1792.52,t0+1802.52,"GYO")
#ly,n2 = caber.input_cuttime(t0+1842.96,t0+1852.96,"GYO")
#bgy,beta_sy = caber.gyo_calib((n1+n2)/2*intv,ly[1],ly_[1],90 * D2R)
#print("GYO CALIBRATION RESULTS:")
#print("x axis->Const bias[%s],Scale factor bias[%s]" % (bgx,beta_sx))
#print("y axis->Const bias[%s],Scale factor bias[%s]" % (bgy,beta_sy))
#print("z axis->Const bias[%s],Scale factor bias[%s]" % (bgz,beta_sz))

##############################################
## alignment of Accelerometer and Gyroscope ##
##############################################

# aligner = align.Align(imu_data)
# aligner.align_mode(-1)
# chkalg = chkimu.Chkalg(aligner.alg_mat("ALL"))
# #chkalg.plot("ALL")
# print(aligner.alg_mat("ALL"))


        
####################################
## Pure IMU Solution & validation ##
####################################

def plotData(timdata,data,dtype):
    timdata = np.asarray(timdata)
    data = np.asarray(data)
    # Remove horizontal space between axes
    if dtype == "LOC":
        fig, axs = plt.subplots(nrows=2, ncols=1, sharex=True)
        fig.suptitle('Pure IMU Location Results(Compare with Reference)')
        plt.xlabel("Time")
        axs[0].set_ylabel("h(m)")
        axs[0].plot(timdata, data[:,2], label = "h")
        axs[1].set_ylabel("lat/lon(degree)")
        axs[1].plot(timdata, data[:,0], label = "lat")
        axs[1].plot(timdata, data[:,1], label = "lon")
        axs[0].legend(loc = "best")
        axs[1].legend(loc = "best")
    else:
        fig, ax = plt.subplots(nrows=1, ncols=1)
        if dtype == "VEL":
            fig.suptitle('Pure IMU Velocity Results(Compare with Reference)')
            ax.set_ylabel("Vn/Ve/Vd(m/s)")
            ax.plot(timdata, data[:,0], label = "Vn")
            ax.plot(timdata, data[:,1], label = "Ve")
            ax.plot(timdata, data[:,2], label = "Vd")
        elif dtype == "POS":
            fig.suptitle('Pure IMU Posture Results(Compare with Reference)')
            ax.set_ylabel("Phi/Theta/Psi(degree)")
            ax.plot(timdata, data[:,0], label = "Phi")
            ax.plot(timdata, data[:,1], label = "Theta")
            ax.plot(timdata, data[:,2], label = "Psi")
        plt.xlabel("Time")
        ax.legend(loc = "best")
    plt.show()
def fread(fp):
    with open(fp) as f:
        f.readline()
        f.readline()
        for line in f:
            linedata = line.split()
            tim = float(linedata[0])
            loc_n = [float(i) for i in linedata[1:4]]
            vel_n = [float(i) for i in linedata[4:7]]
            euler = [float(i) for i in linedata[7:]]
            yield tim, euler, vel_n, loc_n
            
# ts = 91620.0
# inloc = [23.1373950708 * glv.D2R, 113.3713651222 * glv.D2R, 2.175]
# invel = [0,0,0]
# inpos = [0.0107951084511778 * glv.D2R, -2.14251290749072 * glv.D2R, -75.7498049314083 * glv.D2R]

# imudata = imu.ImuData(0.005)
# imudata.load_initstatus(ts, inloc, invel, inpos)

# imuupd = imu.ImuUpd(imudata)
# imuupd.load_initstatus("./data/Data1.bin")

# # validation
# timeList = []
# dlocList = []
# dposList = []
# dvelList = []



# with open("./data/Data1_PureINS.bin","rb") as f:
#     # for tim, euler,vel_n,loc_n in imuupd.updepoch():
#     for tim,euler,vel_n,loc_n in fread("./data/pureimu_results.txt"):
#         refdata = [] # time(sec), lat(rad), lon(rad), h(m), vn(m/s), ve(m/s), vd(m/s), phi(rad), theta(rad), psi(rad)
        
#         for i in range(10):
#             data = f.read(8)
#             data_float = struct.unpack("d", data)[0]
#             refdata.append(data_float)
#         # mloc = np.array([loc_n[0] * glv.R2D, loc_n[1] * glv.R2D, loc_n[2]])
#         mloc = np.array(loc_n)
#         meuler = np.array(euler) * glv.R2D
#         dloc = mloc - np.array(refdata[1:4])
#         dvel = vel_n - np.array(refdata[4:7])
#         dpos = meuler - np.array(refdata[7:])
#         timeList.append(tim)
#         dlocList.append(dloc)
#         dposList.append(dpos)
#         dvelList.append(dvel)
#         print("======================")
#         print("Porcessing Time  : ",tim)
#         # print(tim, mloc, vel_n, euler);

# plotData(timeList,dlocList,"LOC")
# plotData(timeList,dposList,"POS")
# plotData(timeList,dvelList,"VEL")

##########################################
## Loose IMU-Gnss Solution & validation ##
##########################################
m_config = conf.Config()
imudata = imu.ImuData(m_config.intv)
imudata.load_initstatus(m_config.start_time, m_config.init_loc, m_config.init_vel, m_config.init_pos)

imuupd = imu.ImuUpd(imudata)
imuupd.load_initstatus("./data/A15_imu.bin")

# validation
# timeList = []
# dlocList = []
# dposList = []
# dvelList = []



with open("./results/m_sol.txt","w") as fw:
    for x, Qx in imuupd.updepoch():
    # for tim,euler,vel_n,loc_n in fread("./data/ref.txt"):
        # refdata = [] # time(sec), lat(rad), lon(rad), h(m), vn(m/s), ve(m/s), vd(m/s), phi(rad), theta(rad), psi(rad)
        # for i in range(10):
        #     data = f.read(8)
        #     data_float = struct.unpack("d", data)[0]
        #     refdata.append(data_float)
        # mloc = np.array([loc_n[0] * glv.R2D, loc_n[1] * glv.R2D, loc_n[2]])
        tim = imuupd.tim[-1]
        mloc = np.array(imuupd.loc[-1])
        mloc[:2] *= glv.R2D
        mvel = np.array(imuupd.vel[-1])
        mpos = np.array(imuupd.pos[-1]) * glv.R2D
        mpos[-1] += 360
        fw.write("%10.3f %16.10f %16.10f %10.3f %10.3f %10.3f %10.3f %13.8f %13.8f %13.8f\n" % (
            tim, *mloc, *mvel, *mpos
            ))
        print("======================")
        print("Porcessing Time  : ", tim)

plotData(imuupd.tim,imuupd.loc,"LOC")
plotData(imuupd.tim,imuupd.vel,"VEL")
plotData(imuupd.tim,imuupd.pos,"POS")
