# -*- coding: utf-8 -*-
"""
Created on Fri Apr 17 12:52:14 2020

@author: Mocki

E-mail : 605095234@qq.com

TO : Art is piece of luxury
"""
import com
import struct
import glv
import numpy as np
import config as conf
import cood

class ImuData():
    def __init__(self, intv):
        self.intv = intv
        self.g = None     # m/s^2
        self.imu_tim = [] # time flag
        self.imu_gyo = [] # gx,gy,yz (rad)
        self.imu_acc = [] # ax,ay,az (m/s)
        
        self.init_status = 0
        self.init_ts     = None
        self.init_loc    = None
        self.init_vel    = None
        self.init_pos    = None
        
    def load_initstatus(self, ts, loc, vel, pos):
        '''
        Parameters
        ----------
        ts : float
            init_time, sec
        loc : list
            init_location, [lat,lon,h] (rad,m)
        vel : list
            init_velocity
        pos : list
            init_posture, [roll,pitch,heading] (rad)
        Returns
        -------
        None.

        '''
        self.init_ts  = ts
        self.init_loc = np.asarray(loc)
        self.init_vel = np.asarray(vel)
        self.init_pos = np.asarray(pos)
        
        self.g = com.getgravity(pos[0],pos[2])
        
        self.init_status = 1
    
        
    def loadf(self,fp):
        str2float = lambda x : [float(i) for i in x] 
        with open(fp,'r') as fr:
            for line in fr:
                if not line:
                    continue
                data = str2float(line.rstrip().split())
                self.__load(data[0],data[1:4],data[4:])
        print("IMU DATA LOADED.")
        
    def loadbinf(self,binfp):
        with open(binfp,'rb') as f:
            while 1:
                data = []
                try:
                    for i in range(7):
                        data_bin = f.read(8)
                        data_float = struct.unpack("d",data_bin)[0]
                        data.append(data_float)
                    self.__load(data[0],data[1:4],data[4:])
                except Exception:
                    break
        print("IMU DATA LOADED.")
    
    def loadbinepoch(self,binfp):
        '''

        Parameters
        ----------
        binfp : str
            bin file path of imu data.

        Yields
        ------
        list
            t, ls_g[gx, gy, gz], ls_a[ax, ay, az]

        '''
        with open(binfp,'rb') as f:
            while 1:
                data = []
                try:
                    for i in range(7):
                        data_bin = f.read(8)
                        data_float = struct.unpack("d",data_bin)[0]
                        data.append(data_float)
                    yield self.__load(data[0],data[1:4],data[4:])
                except Exception:
                    break
        print("IMU DATA LOADED.")
        
    def __load(self,t,ls_g,ls_a):
        '''
        t   : time flag            [double]
        v_g : vector of Gyro output[list]
        v_a : vector of accelerator[list]
        '''
        te = self.imu_tim[-1] if self.imu_tim else t
        njump = np.round((t - te) / self.intv)
        if njump > 1:
            for it in range(njump - 1):
                self.imu_tim.append(te + (it + 1) * self.intv)
                self.imu_gyo.append([np.nan] * 3)
                self.imu_acc.append([np.nan] * 3)
        self.imu_tim.append(t)
        self.imu_gyo.append(ls_g)
        self.imu_acc.append(ls_a)
        return t, ls_g, ls_a
        
    def imu_mat(self,mat_type):
        if mat_type == "TIM":
            return np.array(self.imu_tim)
        elif mat_type == "GYO":
            return np.array(self.imu_gyo)
        elif mat_type == "ACC":
            return np.array(self.imu_acc)
        elif mat_type == "ALL":
            return np.array([[i] + j + k for i,j,k in zip(self.imu_tim, self.imu_gyo, self.imu_acc)])
        else:
            raise Exception("Wrong MAT TYPE[%s]" % mat_type)
            
    def imu_avg(self,ts,te,dtype): # avg, n
        "average of a period of imu data"
        t0 = self.imu_tim[0]
        ns = int(np.floor((ts - t0) / self.intv))
        ne = int(np.ceil((te - t0) / self.intv))
        if dtype == "ACC": # acc outputs : v (m/s) -> a (m/s^2) = v / intv 
            return np.sum(np.array(self.imu_acc[ns:ne]) / self.intv, axis = 0) / (ne - ns), (ne - ns)
        elif dtype == "GYO": # gyo outputs : theta (rad) -> omega (rad/s) = theta / intv
            return np.sum(np.array(self.imu_gyo[ns:ne]) / self.intv, axis = 0) / (ne - ns), (ne - ns)
        
    def imu_sum(self,ts,te,dtype): # sum, n
        "sum of a period of imu data"
        t0 = self.imu_tim[0]
        ns = int(np.floor((ts - t0) / self.intv))
        ne = int(np.ceil((te - t0) / self.intv))
        if dtype == "ACC": # acc outputs : v (m/s)
            return np.sum(np.array(self.imu_acc[ns:ne]), axis = 0), (ne - ns)
        elif dtype == "GYO": # gyo outputs : theta (rad)
            return np.sum(np.array(self.imu_gyo[ns:ne]), axis = 0), (ne - ns)
        
    def imu_std(self): # for calibration
        A = np.array([[self.g, -self.g,      0,       0,      0,       0],
                      [     0,       0, self.g, -self.g,      0,       0],
                      [     0,       0,      0,       0, self.g, -self.g],
                      [     1,       1,      1,       1,      1,       1]])
        return A
    
    def imu_alg(self): # for alignment
        # navigation frame
        phi = self.pos[0]
        gama_n = np.array([0, 0, -self.g])
        omega_n = np.array([glv.EARTH_ROTATE * np.cos(phi), 0, - glv.EARTH_ROTATE * np.sin(phi)])
        v_n = np.cross(gama_n,omega_n)
        N = np.zeros((3,3))
        N[:,0] = gama_n
        N[:,1] = omega_n
        N[:,2] = v_n
        
        return N
        
    def __iter__(self):
        "return -> [timeflag,gx,gy,gz,ax,ay,az]"
        for i,t in enumerate(self.imu_tim):
            yield [t,*self.imu_gyo[i],*self.imu_acc[i]]

    def __call__(self, t, t_type):
        if t_type == "TIM":
            if t not in self.imu_tim:
                print("Time[%s] not find in timeflag of IMU-DATA." % t)
                return
            else:
                t_index = self.imu_tim.index(t)
                rline = "TIME[%s] : \n\tGyro x/y/z : %10.8f %10.8f %10.8f\n\tAccl x/y/z : %10.8f %10.8f %10.8f" % (t,*self.imu_gyo[t_index],*self.imu_acc[t_index])
                print(rline)
                return (t,*self.imu_gyo[t],*self.imu_acc[t])
        elif t_type == "EPO":
            rline = "TIME[%s] : \n\tGyro x/y/z : %10.8f %10.8f %10.8f\n\tAccl x/y/z : %10.8f %10.8f %10.8f" % (self.imu_tim[t],*self.imu_gyo[t],*self.imu_acc[t])
            print(rline)
            return (self.imu_tim[t],*self.imu_gyo[t],*self.imu_acc[t])
        
class ImuUpd():
    def __init__(self, imudata):
        if not imudata.init_status:
            raise Exception("Error! IMU data needs init status.")
        self.imudata = imudata
        self.binf = None
        self.gnssf = "./data/gnss.txt"
        
        self.iep = 0
        self.tim = []
        self.loc = []
        self.vel = []
        self.pos = []
        
        self.gyo_bias = []
        self.acc_bias = []
        self.gyo_fact = []
        self.acc_fact = []
        
        self.rot_ = None
        self.qua_ = None
        self.rot = None # Rotation matrix
        self.qua = None # Quaternion
        
        self.config = conf.Config()
        self.qw = self.__sys_noise_cov()
        
    def load_initstatus(self, fp):
        self.binf = fp
        self.tim.append(self.imudata.init_ts)
        self.loc.append(self.imudata.init_loc)
        self.vel.append(self.imudata.init_vel)
        self.pos.append(self.imudata.init_pos)
        
        self.gyo_bias = self.config.GYO_BIA_VAL
        self.acc_bias = self.config.ACC_BIA_VAL
        self.gyo_fact = self.config.GYO_SCF_VAL
        self.acc_fact = self.config.ACC_SCF_VAL
        
        self.rot = com.euler2rotation(self.pos[-1])
        self.qua = com.euler2quater(self.pos[-1])
        
        self.Qx = np.zeros((9,9))
        self.load_gnssstatus()
        
    def load_gnssstatus(self):
        self.gtim = []
        self.gloc = []
        self.gloc_std = []
        with open(self.gnssf) as f:
            for line in f:
                linedata = [float(i) for i in line.split()]
                self.gtim.append(linedata[0])
                self.gloc.append([linedata[1] * glv.D2R, linedata[2] * glv.D2R, linedata[3]])
                self.gloc_std.append(linedata[4:])
                
    def find_gpsref_data_index(self, index, reftime):
        n = len(self.gtim)
        for i in range(index, n):
            t = self.gtim[i]
            if t - reftime < self.imudata.intv and t > reftime:
                return 1, i
            elif t > reftime:
                return 0, i
        return 0, -1
    
    def antenna_cor(self, imuloc, gloc):
        Dr_I = com.DrMat(imuloc, 1)
        gloc_c = gloc - Dr_I @ self.rot @ self.config.antenna_arm
        return gloc_c
    
    def updepoch_imu(self):
        for t,ls_g,ls_a in self.imudata.loadbinepoch(self.binf):
            if t < self.imudata.init_ts:
                continue
            self.iep += 1
            if self.iep == 1:
                # yield self.tim[-1], self.pos[-1], self.vel[-1], self.loc[-1]
                continue
            self.tim.append(t)
            loc_p = self.loc[-1]
            vel_p = self.vel[-1]
            
            if self.iep == 2:
                vel_m = vel_p
                loc_m = loc_p
            else:
                vel_m = vel_p + (vel_p - self.vel[-2]) / 2
                loc_m = loc_p + (loc_p - self.loc[-2]) / 2
            
            self.__cor_imu()
            
            omega_n_ie, omega_n_en = com.earthrotatvec(loc_m, vel_m)
            grav_n = com.getgravity(loc_m[0], loc_m[2])
            
            # update vel
            vel_n = self.__updvel(vel_m, omega_n_ie, omega_n_en, grav_n)
            self.vel.append(vel_n)
            
            # update loc
            loc_n = self.__updloc()
            self.loc.append(loc_n)
            
            # update pos
            vel_m = (vel_n + vel_p) / 2
            loc_m = (loc_n + loc_p) / 2
            omega_n_ie, omega_n_en = com.earthrotatvec(loc_m, vel_m)
            q_b_n = self.__updpos(omega_n_ie, omega_n_en)
            
            self.qua_ = np.copy(self.qua)
            self.rot_ = np.copy(self.rot)
            self.qua = q_b_n
            self.rot = com.quater2rotation(q_b_n)
            
            pos_n = com.rotation2euler(self.rot)
            self.pos.append(pos_n)
            
            yield t, pos_n, vel_n, loc_n
    
    def updepoch(self):
        x  = np.asarray(self.config.init_x)
        Qx = np.asarray(self.config.init_Qx)
        gps_refdata_index = 0
        for t, pos_n, vel_n, loc_n in self.updepoch_imu():
            cloc_n = np.copy(loc_n)
            dx = np.zeros(3 * 7)
            ok, gps_refdata_index = self.find_gpsref_data_index(gps_refdata_index, t)
            if ok:
                # update status
                PHI, Qw = self.get_status_equation_matrix()
                dx_ = PHI @ dx
                Qx_ = PHI @ Qx @ PHI.T + Qw
                gnss_pureloc = np.asarray(self.gloc[gps_refdata_index])
                gloc_std = np.asarray(self.gloc_std[gps_refdata_index])
                Dr = com.DrMat(cloc_n)
                Hr, Rk = self.get_observation_equation_matrix(gloc_std)
                gloc_c = self.antenna_cor(cloc_n, gnss_pureloc)
                Z = Dr @ (cloc_n - gloc_c)
                S = Hr @ Qx_ @ Hr.T + Rk
                K = Qx_ @ Hr.T @ np.linalg.inv(S)
                dx = dx_ + K @ (Z - Hr @ dx_)
                temp = np.eye(len(x)) - K @ Hr
                Qx = temp @ Qx_ @ temp.T + K @ Rk @ K.T
            else:
                PHI, Qw = self.get_status_equation_matrix()
                dx = PHI @ dx
                Qx = PHI @ Qx @ PHI.T + Qw
            Rm,Rn = com.mcucradius(cloc_n[0])
            dx[0] /= Rm
            dx[1] /= (Rn * np.cos(cloc_n[0]))
            dx[2] /= -1
            self.loc[-1] -= dx[:3]
            self.vel[-1] -= dx[3:6]
            self.pos[-1] = com.rotation2euler(np.linalg.inv(com.rv2m(dx[6:9])) @ self.rot)
            self.gyo_bias += dx[9:12]
            self.gyo_fact += dx[12:15]
            self.acc_bias += dx[15:18]
            self.acc_fact += dx[18:]
            self.qua = com.euler2quater(self.pos[-1])
            self.rot = com.quater2rotation(self.qua)
            yield x, Qx
            
    
    def get_status_equation_matrix(self):
        dt = self.imudata.imu_tim[-1] - self.imudata.imu_tim[-2]
        gc = self.__sys_control_mat()
        PHI = self.__transfer_mat()
        Qw = 0.5 * (PHI @ gc @ self.qw @ gc.T @ PHI.T + gc @ self.qw @ gc.T) * dt
        return PHI, Qw
    
    def get_observation_equation_matrix(self, loc_std):
        lb = self.config.antenna_arm
        Hr = np.zeros((3, 3*7))
        Hr[:,:3] = np.identity(3)
        Hr[:,6:9] = self.rot @ com.antisym(lb)
        Rk = np.diag(loc_std**2)
        return Hr, Rk
    
    def __cor_imu(self):
        dt = self.imudata.imu_tim[-1] - self.imudata.imu_tim[-2]
        self.imudata.imu_acc[-1] = (self.imudata.imu_acc[-1] - self.acc_bias * dt) # / (1 + self.acc_fact)
        self.imudata.imu_gyo[-1] = (self.imudata.imu_gyo[-1] - self.gyo_bias * dt) # / (1 + self.gyo_fact)
        
    def __updpos(self, omega_n_ie, omega_n_en):
        dt = self.imudata.imu_tim[-1] - self.imudata.imu_tim[-2]
        
        ls_gp = np.asarray(self.imudata.imu_gyo[-2])
            
        ls_g = np.asarray(self.imudata.imu_gyo[-1])
        # update b coor
        eqv_phi_b = ls_g + com.antisym(ls_gp) @ ls_g / 12
        q_phi_b   = com.rotvec2quater(eqv_phi_b)
        eqv_eta_b = (omega_n_ie + omega_n_en) * dt
        q_eta_b   = com.quaterConjugate(com.rotvec2quater(eqv_eta_b))
        q_b_n = com.quaterMultiply(com.quaterMultiply(q_eta_b, self.qua), q_phi_b)
        q_b_n = com.quater2unit(q_b_n)
        return q_b_n
        
    def __updvel(self, vel_m, omega_n_ie, omega_n_en, grav_n):
        dt = self.imudata.imu_tim[-1] - self.imudata.imu_tim[-2]
        
        ls_ap = np.asarray(self.imudata.imu_acc[-2])
        ls_gp = np.asarray(self.imudata.imu_gyo[-2])
            
        ls_g = np.asarray(self.imudata.imu_gyo[-1])
        ls_a = np.asarray(self.imudata.imu_acc[-1])
            
        eqv_eta_b = (omega_n_ie + omega_n_en) * dt
        
        dvel_gcor_n = (grav_n - com.antisym(2 * omega_n_ie + omega_n_en) @ vel_m) * dt
        dvel_f_b_p  = ls_a + 0.5 * com.antisym(ls_g) @ ls_a \
                            + (com.antisym(ls_gp) @ ls_a + com.antisym(ls_ap) @ ls_g) / 12
        dvel_f_n    = (np.identity(3) - 0.5 * com.antisym(eqv_eta_b)) @ self.rot @ dvel_f_b_p
        vel_n       = self.vel[-1] + dvel_f_n + dvel_gcor_n
        return vel_n
        
        
    def __updloc(self):
        
        dt = self.imudata.imu_tim[-1] - self.imudata.imu_tim[-2]
        vel_p = self.vel[-2]
        vel   = self.vel[-1]
        loc_p = self.loc[-1]
        
        h = loc_p[2] - (vel_p[2] + vel[2]) * dt / 2
        h_bar = (h + loc_p[2]) / 2
        Rm,_ = com.mcucradius(loc_p[0])
        lat = loc_p[0] + (vel_p[0] + vel[0]) / (2 * (Rm + h_bar)) * dt
        lat_bar = (lat + loc_p[0]) / 2
        _,Rn = com.mcucradius(lat_bar)
        lon = loc_p[1] + (vel_p[1] + vel[1]) / (2 * (Rn + h_bar)) / np.cos(lat_bar) * dt
        
        loc_n = np.array([lat, lon, h])
        return loc_n
        
        
        
    def __transfer_mat(self):
        # get transfer matrix : PHI_k/k-1
        dt = self.imudata.imu_tim[-1] - self.imudata.imu_tim[-2]
        
        loc_n = self.loc[-2]
        vel_n = self.vel[-2]
        pos_n = self.pos[-2]
        
        
        lat, lon, h = loc_n
        vn, ve, vd = vel_n
        phi, theta, psi = pos_n
        Rm,Rn = com.mcucradius(lat)
        gcc = com.getgravity(lat, h)
        omega_n_ie, omega_n_en = com.earthrotatvec(loc_n, vel_n)
        omega_n_in = omega_n_ie + omega_n_en
        
        ls_g = np.asarray(self.imudata.imu_gyo[-2])
        ls_a = np.asarray(self.imudata.imu_acc[-2])
        
        Frr = np.array([
            [-vd / (Rm + h), 0, vn / (Rm + h)],
            [ve * np.tan(lat) / (Rn + h), -(vd + vn * np.tan(lat)) / (Rn + h), ve / (Rn + h)],
            [0, 0, 0]])
        Fvr = np.array([
            [-2*ve*glv.EARTH_ROTATE*np.cos(lat) / (Rm + h) - ve**2 / np.cos(lat)**2 / ((Rm + h)*(Rn + h)), 0, vn*vd/((Rm + h)**2) - ve**2*np.tan(lat)/((Rn + h)**2)],
            [2*glv.EARTH_ROTATE*(vn*np.cos(lat) - vd*np.sin(lat))/(Rm + h) + vn*ve / np.cos(lat)**2 / ((Rm + h)*(Rn + h)), 0, (ve*vd + vn*ve*np.tan(lat)) / ((Rn + h)**2)],
            [2*glv.EARTH_ROTATE*ve*np.sin(lat) / (Rm + h), 0, -ve**2/((Rn + h)**2) - vn**2 / ((Rm + h)**2) + 2*gcc[2] / (np.sqrt(Rm * Rn) + h)]
            ])
        Fvv = np.array([
            [vd / (Rm + h), -2*(glv.EARTH_ROTATE * np.sin(lat) + ve*np.tan(lat) / (Rn + h)), vn / (Rm + h)],
            [2*glv.EARTH_ROTATE*np.sin(lat) + ve * np.tan(lat) / (Rn + h), (vd + vn * np.tan(lat)) / (Rn + h), 2*glv.EARTH_ROTATE*np.cos(lat) + ve / (Rn + h)],
            [-2*vn /( Rm + h), -2*(glv.EARTH_ROTATE*np.cos(lat) + ve / (Rn + h)), 0]
            ])
        Fphir = np.array([
            [-glv.EARTH_ROTATE*np.sin(lat) / (Rm + h), 0, ve / ((Rn + h)**2)],
            [0, 0, -vn / ((Rm + h)**2)],
            [-glv.EARTH_ROTATE*np.cos(lat) / (Rm + h) - ve / np.cos(lat)**2 / ((Rm + h)*(Rn + h)), 0, -ve*np.tan(lat) / ((Rn + h)**2)]
            ])
        Fphiv = np.array([
            [0, 1 / (Rn + h), 0],
            [-1 / (Rm + h), 0, 0],
            [0, -np.tan(lat) / (Rn + h), 0]
            ])
        
        F = np.zeros((3 * 7, 3 * 7))
        F[:3,:3] =  Frr
        F[:3,3:6] = np.identity(3)
        F[3:6,:3] = Fvr
        F[3:6,3:6] = Fvv
        F[6:9,:3] = Fphir
        F[6:9,3:6] = Fphiv
        F[3:6,6:9] = com.antisym(self.rot_ @ ls_a)
        F[6:9,6:9] = -com.antisym(omega_n_in)
        F[6:9,9:12] = -self.rot_
        F[3:6,12:15] = self.rot_
        F[3:6,-3:] = self.rot_ @ np.diag(ls_a/dt)
        F[6:9,-6:-3] = -self.rot_ @ np.diag(ls_g/dt)
        F[9:12,9:12] = - 1 / self.config.GYO_BIA_TAU * np.identity(3)
        F[12:15,12:15] = - 1 / self.config.ACC_BIA_TAU * np.identity(3)
        F[-6:-3,-6:-3] = - 1 / self.config.GYO_SCF_TAU * np.identity(3)
        F[-3:,-3:] = - 1 / self.config.ACC_SCF_TAU * np.identity(3)
        return np.identity(21) + F * dt
    
    def __sys_noise_cov(self):
        qw = np.zeros((21, 21))
        qw[:3,:3]       = self.config.GYOARW**2 * np.identity(3)
        qw[3:6,3:6]     = self.config.ACCVRW**2 * np.identity(3)
        
        qw[9:12,9:12]     = 2*self.config.GYO_BIA_STD**2 / self.config.GYO_BIA_TAU * np.identity(3)
        qw[12:15,12:15]   = 2*self.config.ACC_BIA_STD**2 / self.config.ACC_BIA_TAU * np.identity(3)
        qw[-6:-3,-6:-3] = 2*self.config.GYO_SCF_STD**2 / self.config.GYO_SCF_TAU * np.identity(3)
        qw[-3:,-3:]     = 2*self.config.ACC_SCF_STD**2 / self.config.ACC_SCF_TAU * np.identity(3)
        return qw
    
    def __sys_control_mat(self):
        gc = np.zeros((21, 21))
        gc[:3,:3] = self.rot_
        gc[3:6,3:6] = -self.rot_
        gc[9:12,9:12] = np.identity(3)
        gc[12:15,12:15] = np.identity(3)
        gc[15:18,15:18] = np.identity(3)
        gc[18:,18:] = np.identity(3)
        return gc