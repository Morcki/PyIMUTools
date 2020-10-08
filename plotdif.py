# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 16:14:58 2020

@author: mocki
"""
import matplotlib.pyplot as plt
import numpy as np

fref = "./data/ref.txt"
fsol = "./results/m_sol.txt"


dif = []
with open(fref) as f1, open(fsol) as f2:
    f2.readline()
    for line_ref in f1:
        refdata = [float(i) for i in line_ref.split()]
        try:
            soldata = [float(i) for i in f2.readline().split()]
            dif.append([i - j for i,j in zip(soldata, refdata)])
        except Exception:
            break
dif = np.asarray(dif)
for i in range(8):
    plt.plot(dif[:,i+1])