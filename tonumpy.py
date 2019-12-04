#!/usr/bin/python
# coding: UTF-8
 
import numpy as np
from pypcd import pypcd, numpy_pc2

pc_msg = pypcd.PointCloud.from_path('/home/liu/bag/urban/urban01.pcd').to_msg()
pc = numpy_pc2.pointcloud2_to_array(pc_msg)
data = np.zeros([pc.shape[1],3])

idx = 0
for i in pc[0]:
    data[idx, 0] = i[0]
    data[idx, 1] = i[1] 
    data[idx, 2] = i[2]
    idx += 1
np.save('urban01.npy',data)