# -*- coding: utf-8 -*-
"""
Created on Mon Dec 10 15:34:03 2018

@author: SongXiaocheng
"""

import numpy as np
lineList = []
tmpMesh = []
with open('data/test-mesh.txt', 'r') as f:
    for line in f:
        if line.strip()=="":
            if len(tmpMesh)!=0:
                lineList.append(np.array(tmpMesh));
                tmpMesh = []
        else:
            index = line.strip().split(' ')
            tmpMesh.append([float(index[0]),float(index[1])])
    if len(tmpMesh)!=0:
        lineList.append(np.array(tmpMesh));
         
for line in lineList:
    print(line)