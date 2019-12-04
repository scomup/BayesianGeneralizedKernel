#!/usr/bin/python3
# coding: UTF-8
 
import numpy as np
import matplotlib.pyplot as plt
from math import *
import random
import time
from suface_pub import * 



def get_test_data(sigma):
    x = random.uniform(0, 2*np.pi)
    y = np.cos(x)
    #y_prime = random.normalvariate(y, sigma)
    return x, y

class Map3D():
    def __init__(self, world_size, resolution, radius=0.2):
        cH = int(world_size / resolution)
        cW = int(world_size / resolution)
        H = cH * 2
        W = cW * 2
        self.map_size = np.array([H, W])
        self.offset = np.array([cH, cW])
        self.raw_data = np.zeros([H, W])
        self.resolution = resolution
        self.radius = radius
        x, y = np.meshgrid(range(-cH, cH),range(-cW, cW))
        self.location = np.dstack([x*resolution,y*resolution])
        self.elevation = np.full_like(self.raw_data, 0) 
        self.lambdas = np.full_like(self.raw_data, 0) 
        self.raw_points = []
    
    def points(self):
        x, y = np.meshgrid(range(-self.offset[0], self.offset[0]),
            range(-self.offset[1], self.offset[1]))
        p = np.dstack([x*self.resolution,y*self.resolution,self.elevation])
        p = p.reshape([-1,3])
        l = self.lambdas.reshape([-1])
        return p[l>0,:]


    def w2m(self, w):
        m = w[0:2]/self.resolution + self.offset
        return np.round(m)
        

    def m2w(self, m):
        w = m - self.offset
        i = w[:,0] * self.resolution
        j = w[:,1] * self.resolution
        w = np.array([i, j])
        return w

    def sparse_kernel(self, x, x_star, l = 0.5):
        diff = x_star - x[0:2].reshape(2,1)
        dist = np.linalg.norm(diff, axis=0)
        r = dist/l
        pi = np.pi
        k = (2 + np.cos(2*pi*r))/3*(1-r) + np.sin(2*np.pi*r)/(2*np.pi)
        k[dist>=l] = np.finfo(float).eps
        return k

    def points_in_circle(self, radius, x0=0, y0=0):
        x_ = np.arange(x0 - radius+1 , x0 + radius-1 , dtype=int)
        y_ = np.arange(y0 - radius+1 , y0 + radius-1 , dtype=int)
        x, y = np.where((x_[:,np.newaxis] - x0)**2 + (y_ - y0)**2 <= radius**2)
        points = []
        for x, y in zip(x_[x], y_[y]):
            if(x < 0 or x >= self.map_size[1]):
                continue
            if(y < 0 or y >= self.map_size[0]):
                continue
            points.append([x,y])
        return np.array(points)

    def update(self, point):
        self.raw_points.append(point)
        m = self.w2m(point)
        points_m = self.points_in_circle(self.radius/self.resolution, x0 =  m[0], y0 =  m[1])
        if(points_m.size == 0):
            return
        points_w = self.m2w(points_m)
        k = self.sparse_kernel(point, points_w)
        
        lam = self.lambdas[points_m[:,1], points_m[:,0]]
        mu_0 = self.elevation[points_m[:,1], points_m[:,0]]
        y_star = (lam * mu_0 + k * point[2])/(lam + k)
        max_z = np.max(y_star)
        z = point[2]

        #if(np.abs(max_z -z) > 1):
        #    pass
        #    max_z = np.max(y_star)
        #    z = point[2]
        #    print(max_z, z)
        self.elevation[points_m[:,1], points_m[:,0]] = y_star
        self.lambdas[points_m[:,1], points_m[:,0]] = lam + k

sigma = 0.1
def get_test_data(sigma):
    x = random.uniform(0, 2*np.pi)
    y = np.cos(x)
    if x > 3:
        y = 0
    y_prime = random.normalvariate(y, sigma)
    return x, y_prime


if __name__ == '__main__':
    #points=[]
    #for i in range(1000):
    #    x, y = get_test_data(sigma)
    #    points.append([0,x,y])
    #points = np.array(points)

    points = np.load('urban01.npy')
    map3d = Map3D(40, resolution = 0.05, radius = 0.3)
    mp = MarkerPub()
    mp.start()
    show = 0
    for point in points:
        #time.sleep(0.01)
        #if(point[1] < 9 or point[1] > 11):
        #    continue
        #if(point[0] < 0 or point[0] > 5):
        #    continue

        map3d.update(point)
        show += 1
        if(show == 5000):
            show = 0
            mp.pub_data(map3d.points())
            mp.pub_data(np.array(map3d.raw_points),'raw')
            time.sleep(0.1)
    mp.pub_data(map3d.points())
    time.sleep(1)






