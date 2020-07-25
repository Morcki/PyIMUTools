# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 22:03:15 2020

@author: mocki
"""
import glv
import numpy as np

def norm(x):
    return np.sqrt(x @ x)

def antisym(v):
    '''
    Parameters
    ----------
    v : list (3 entries)
        1x3 vector

    Returns
    -------
    Antisymmetric matrix:
    '''
    if not len(v) == 3:
        raise Exception("Vector length large than 3!")
    v_ = np.asarray(v)
    vx = np.array([
        [0, -v_[2], v_[1]],
        [v_[2], 0, -v_[0]],
        [-v_[1], v_[0], 0]
        ])
    return vx

def quaterAnti(q):
    if not len(q) == 4:
        raise Exception("Quaternion length large than 4!")
    q_ = np.asarray(q)
    qx = np.array([
        [q_[0], -q_[1], -q_[2], -q_[3]],
        [q_[1],  q_[0], -q_[3],  q_[2]],
        [q_[2],  q_[3],  q_[0], -q_[1]],
        [q_[3], -q_[2],  q_[1],  q_[0]]
        ])
    return qx

def quaterConjugate(q):
    q  = np.asarray(q) 
    qc = np.copy(q)
    qc[1:] *= -1
    return qc
    
def quaterMultiply(q1, q2):
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)
    return quaterAnti(q1) @ q2

def euler2quater(euler):
    '''
    euler angles : 
        [roll, pitch, yaw(heading)], righthand
        roll  -> phi
        pitch -> theta
        yaw   -> psi
    quaternion : 
        q0, q1, q2, q3
    '''
    phi, theta, psi     = euler
    sphi_2,cphi_2       = np.sin(phi/2),np.cos(phi/2)
    stheta_2,ctheta_2   = np.sin(theta/2),np.cos(theta/2)
    spsi_2,cpsi_2       = np.sin(psi/2),np.cos(psi/2)
    
    
    q0 = cphi_2*ctheta_2*cpsi_2 + sphi_2*stheta_2*spsi_2
    q1 = sphi_2*ctheta_2*cpsi_2 - cphi_2*stheta_2*spsi_2
    q2 = cphi_2*stheta_2*cpsi_2 + sphi_2*ctheta_2*spsi_2
    q3 = cphi_2*ctheta_2*spsi_2 - sphi_2*stheta_2*cpsi_2
    q = quater2unit(np.array([q0,q1,q2,q3]))
    return q

def rotvec2quater(eqv):
    eqv = np.asarray(eqv)
    meqv_2 = norm(eqv/2)
    q = np.zeros(4)
    if np.abs(meqv_2) < 1e-9:
        return q
    else:
        q[0] = np.cos(meqv_2)
        q[1:] = np.sin(meqv_2) / meqv_2 * eqv / 2
        return q

def euler2rotation(euler):
    phi, theta, psi = euler
    sphi,cphi       = np.sin(phi),np.cos(phi)
    stheta,ctheta   = np.sin(theta),np.cos(theta)
    spsi,cpsi       = np.sin(psi),np.cos(psi)
    
    r = np.array([
        [ctheta*cpsi, -cphi*spsi + sphi*stheta*cpsi, sphi*spsi + cphi*stheta*cpsi],
        [ctheta*spsi, cphi*cpsi + sphi*stheta*spsi , -sphi*cpsi + cphi*stheta*spsi],
        [-stheta    , sphi*ctheta                  , cphi*ctheta]
        ])
    return r

def quater2rotation(q):
    q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
    r = np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2 * (q1*q2 - q0*q3)         , 2 * (q0*q2 + q1*q3)],
        [2 * (q0*q3 + q1*q2)          , q0**2 - q1**2 +q2**2 - q3**2, 2 * (q2*q3 - q0*q1)],
        [2 * (q1*q3 - q0*q2)          , 2 * (q0*q1 + q2*q3)         , q0**2 - q1**2 - q2**2 + q3**2]
        ])
    return r

def rotvec2rotation(eqv):
    '''
    eqv :
        eqv_x, eqv_y, eqv_z
    '''
    eqv = np.asarray(eqv)
    meqv = norm(eqv)
    anti_eqv = antisym(eqv)
    r = np.identity(3) + np.sin(meqv) / meqv * anti_eqv + (1 - np.cos(meqv)) / meqv**2 * (anti_eqv @ anti_eqv)
    return r

def rotation2euler(r):
    phi   = np.arctan2(r[2][1], r[2][2])
    theta = np.arctan(-r[2][0]/np.sqrt(r[2][1]**2 + r[2][2]**2))
    psi   = np.arctan2(r[1][0], r[0][0])
    euler = np.array([phi, theta, psi])
    return euler

def quater2unit(q):
    q = np.asarray(q)
    mq = norm(q)
    return q / mq

def mcucradius(lat):
    '''

    Parameters
    ----------
    lat : float(rad)
        latitude of local

    Returns
    -------
    Rm,Rn
        Radius of meridian circle & Radius of unitary circle
    '''
    dump = 1 - glv.E2_WGS84 * np.sin(lat) ** 2
    Rm = glv.RE_WGS84 * (1 - glv.E2_WGS84) / dump ** 1.5
    Rn = glv.RE_WGS84 / np.sqrt(dump)
    return Rm, Rn

def earthrotatvec(loc,vel):
    lat = loc[0]
    h   = loc[2]
    vn  = vel[0] 
    ve  = vel[1]
    Rm,Rn = mcucradius(lat)
    omega_n_ie = np.array([glv.EARTH_ROTATE * np.cos(lat), 0, -glv.EARTH_ROTATE * np.sin(lat)])
    omega_n_en = np.array([ve / (Rn + h), -vn / (Rm + h), -ve * np.tan(lat) / (Rn + h)])
    return omega_n_ie, omega_n_en

def getgravity(lat, h):
    sinlat = np.sin(lat)
    coslat = np.cos(lat)
    m = glv.EARTH_ROTATE**2 * glv.RE_WGS84**2 * glv.BE_WGS84 / glv.GM
    gama_lat = (glv.RE_WGS84 * glv.GAMA_A * coslat**2 + glv.BE_WGS84 * glv.GAMA_B * sinlat**2) \
                        / np.sqrt(glv.RE_WGS84**2 * coslat**2 + glv.BE_WGS84**2 * sinlat**2)
    gama = gama_lat * (1 - \
                           2 / glv.RE_WGS84 * (1 + glv.FR_WGS84 + m - 2 * glv.FR_WGS84 * sinlat**2) * h \
                               + 3 * h**2 / glv.RE_WGS84**2)
    gama_n = np.array([0 , 0, gama])
    return gama_n


def DrMat(loc, itype = 0):
    Rm, Rn = mcucradius(loc[0])
    if not itype:
        Dr = np.diag([Rm + loc[2], (Rn + loc[2]) * np.cos(loc[0]), -1])
    else:
        Dr = np.diag([1 / (Rm + loc[2]), 1 / ((Rn + loc[2]) * np.cos(loc[0])), -1])
    return Dr


# inpos = [0.0107951084511778 * glv.D2R, -2.14251290749072 * glv.D2R, -75.7498049314083 * glv.D2R]
# qua   = euler2quater(inpos)
# rot   = euler2rotation(inpos)

# rot_  = quater2rotation(qua)

# pos_  = rotation2euler(rot)

# eqv = [0.0107951084511778 * glv.D2R, -2.14251290749072 * glv.D2R, -75.7498049314083 * glv.D2R]

# qua = rotvec2quater(eqv)

# rot = rotvec2rotation(eqv)
# rot_ = quater2rotation(qua)


# eqv = [1,2,3]
# q = rotvec2quater(eqv)
# r = quater2rotation(q)
# euler  = rotation2euler(r)


# print(q)
# print(e)
# print(e_)