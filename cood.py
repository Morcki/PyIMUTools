# -*- coding: utf-8 -*-
"""
Created on Tue Jul 21 11:41:25 2020

@author: mocki
"""
import numpy as np
from numpy.linalg import inv
from numpy import sin,cos,tan,arctan,sqrt,pi

def xyzblh(xyz, scale, a0, b0, dx, dy, dz):  # x,geod是数组（列表），三个元素。scale取1,a0,b0,dx,dy,dz取0
    '''
    input:
        xyz          tuple or list(X,Y,Z)(3) * ECEF  (earth  center  earth  fixed)  coordinates  system
    output:
        geod         tuple or list(B,L,H)(3) 单位：（弧度）
    '''
    geod = [0, 0, 0]
    if a0 == 0 or b0 == 0:
        a = 6378137.0
        b = 298.257223563  # alpha = (a-b)/a
    else:
        a = a0
        b = b0
    xp = xyz[0] * scale + dx
    yp = xyz[1] * scale + dy
    zp = xyz[2] * scale + dz
    if xp == 0 and yp == 0 and zp == 0:
        geod = [0,0,0]
        return(geod)
    if b <= 6000000:
        b = a - a / b
    e2 = (a * a - b * b) / (a * a)
    s = sqrt(xp ** 2 + yp ** 2)
    geod[1] = arctan(yp / xp)
    if geod[1] < 0:
        if yp > 0:
            geod[1] += pi
        if yp < 0:
            geod[1] += 2 * pi
    else:
        if yp < 0:
            geod[1] += pi
    zps = zp / s
    geod[2] = sqrt(xp * xp + yp * yp + zp * zp) - a
    geod[0] = arctan(zps / (1.0 - e2 * a / (a + geod[2])))
    n = 1
    rhd = rbd = 1
    while rbd * n > 1e-4 or rhd > 1e-4:
        n = a / sqrt(1.0 - e2 * sin(geod[0]) * sin(geod[0]))
        tmp1 = geod[0]
        tmp2 = geod[2]
        geod[2] = s / cos(geod[0]) - n
        geod[0] = arctan(zps / (1.0 - e2 * n / (n + geod[2])))
        rbd = abs(tmp1 - geod[0])
        rhd = abs(tmp2 - geod[2])
    return geod
def blhxyz(geod,a0,b0):
    '''
    input:
        geod         tuple or list(B,L,H)(3) 单位：（弧度）
    output:
		x            tuple or list(X,Y,Z)(3)
    '''
    xyz = [0,0,0]
    if a0 == 0 or b0 == 0:
        a = 6378137.0
        b = 298.257223563  # alpha = (a-b)/a
    else:
        a = a0
        b = b0
    if b <= 6000000:
        b = a - a / b
    e2 = 1 - (b ** 2) / (a ** 2)

    W = sqrt(1 - e2 * sin(geod[0]) **2)
    N = a / W
    xyz[0] = (N + geod[2]) * cos(geod[0]) * cos(geod[1])
    xyz[1] = (N + geod[2]) * cos(geod[0]) * sin(geod[1])
    xyz[2] = (N * (1 - e2) + geod[2]) * sin(geod[0])
    return xyz
def xyz2enu(pos):
    "pos : geodetic position [lat,lon,h] (rad)"
    M = np.array([[-sin(pos[1])              ,  cos(pos[1])              , 0          ],
                  [-sin(pos[0]) * cos(pos[1]), -sin(pos[0]) * sin(pos[1]), cos(pos[0])],
                  [ cos(pos[0]) * cos(pos[1]),  cos(pos[0]) * sin(pos[1]), sin(pos[0])]
                  ]) # rotation matrix
    return M

def ecef2enu(r,pos):
    '''
    In:
        r   : vector in ecef coordinate [x,y,z]
        pos : geodetic position [lat,lon,h] (rad)"
    out:
        enu : vector in local tangental coordinate [e,n,u]
    '''
    M = xyz2enu(pos)
    N = np.array(r) # vector in ecef coordinate [x,y,z]
    return M @ N

def enu2ecef(enu,pos):
    '''
    In:
        enu : vector in local tangental coordinate [e,n,u]
        pos : geodetic position [lat,lon,h] (rad)"
    out:
        r   : vector in ecef coordinate [x,y,z]
    '''
    M = xyz2enu(pos)
    N = np.array(enu) # vector in local tangental coordinate [e,n,u]
    return inv(M) @ N

def ned2xyz(pos):
    Cne = np.array([
        [-sin(pos[0])*cos(pos[1]), -sin(pos[1]), -cos(pos[0])*cos(pos[1])],
        [-sin(pos[0])*sin(pos[1]),  cos(pos[1]), -cos(pos[0])*sin(pos[1])],
        [cos(pos[0])             ,            0, -sin(pos[1])]
        ])
    return Cne
