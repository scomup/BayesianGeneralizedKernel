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
        self.traversability = np.full_like(self.raw_data,0)
        self.alpha = np.full_like(self.raw_data,0)
        self.beta = np.full_like(self.raw_data,0)
        self.variance = np.full_like(self.raw_data,0)
        self.raw_points = []
    
    def get_points(self):
        x, y = np.meshgrid(range(-self.offset[0], self.offset[0]),
            range(-self.offset[1], self.offset[1]))
        p = np.dstack([x*self.resolution,y*self.resolution,self.elevation])
        p = p.reshape([-1,3])
        l = self.lambdas.reshape([-1])
        return p[l>0,:]

    def get_traversability(self):
        l = self.lambdas.reshape([-1])
        t = self.traversability.reshape([-1,1])
        #t = self.variance.reshape([-1,1])
        return np.sqrt(t[l>0,:])
    def w2m(self, w):
        m = w[0:2]/self.resolution + self.offset
        return np.round(m)
        

    def m2w(self, m):
        w = m - self.offset
        i = w[:,0] * self.resolution
        j = w[:,1] * self.resolution
        w = np.array([i, j])
        return w

    def sparse_kernel(self, x, x_star, l = 0.3):
        diff = x_star - x[0:2].reshape(2,1)
        dist = np.linalg.norm(diff, axis=0)
        r = dist/l
        pi = np.pi
        k = (2 + np.cos(2*pi*r))/3*(1-r) + np.sin(2*np.pi*r)/(2*np.pi)
        k[dist>=l] = np.finfo(float).eps
        return k

    def calc_traversability(self, location, elevation):
        points = np.vstack([location, elevation])
        center = np.mean(points, axis=1)
        max_step = np.max(points[2,:]) - np.min(points[2,:])
        cov = np.cov(points, rowvar=1)
        w, v = np.linalg.eig(cov)
        small_idx = np.argmin(w)
        slope = np.arccos(np.abs(v[2, small_idx]))/ np.pi * 180
        if(np.isnan(slope)):
            print(slope)
        h = 0 #step height
        s = 0 #slope
        r = 0 #rough

        h_crit = 0.2
        s_crit = 10.
        r_crit = 0.5

        s = 1. / (1. + np.exp(-(slope - s_crit)))
        h = max_step / h_crit
        r = np.sqrt(cov[2,2] )/ r_crit
        v = 0
        if (h > 1 or s > 1 or r > 1):
            v = 1
        else:
            v = np.min([0.9 * h + 0.105 * s + 0.05 * r, 1.0])
        return s


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
        area = 30
        if(point[0]>area or point[0]<-area):
            return
        if(point[1]>area or point[1]<-area):
            return
        
        self.raw_points.append(point)
        m = self.w2m(point)
        points_m = self.points_in_circle(self.radius/self.resolution, x0 =  m[0], y0 =  m[1])
        if(points_m.size == 0):
            return
        points_w = self.m2w(points_m)
        
        lam = self.lambdas[points_m[:,1], points_m[:,0]]
        mu_0 = self.elevation[points_m[:,1], points_m[:,0]]
        var = self.variance[points_m[:,1], points_m[:,0]]
        #v_cur = np.max(var)
        #w = np.exp(-v_cur*20)
        k = self.sparse_kernel(point, points_w, 0.3 )
        y_star = (lam * mu_0 + k * point[2])/(lam + k)
        self.variance[points_m[:,1], points_m[:,0]] = (lam/(lam + k))*(var + k*(mu_0 - point[2])**2/(lam + k))
        self.elevation[points_m[:,1], points_m[:,0]] = y_star
        self.lambdas[points_m[:,1], points_m[:,0]] = lam + k

        v = self.calc_traversability(points_w, y_star)
        alpha_star = self.alpha[points_m[:,1], points_m[:,0]] + k*v
        beta_star = self.beta[points_m[:,1], points_m[:,0]] + k*(1-v)
        self.traversability[points_m[:,1], points_m[:,0]] = alpha_star/(alpha_star + beta_star)
        self.alpha[points_m[:,1], points_m[:,0]] = alpha_star
        self.beta[points_m[:,1], points_m[:,0]] = beta_star

if __name__ == '__main__':
    #points=[]
    #for i in range(1000):
    #    x, y = get_test_data(sigma)
    #    points.append([0,x,y])
    #points = np.array(points)

    #points = np.load('real.npy')
    points = np.load('urban01.npy')
    map3d = Map3D(40, resolution = 0.1, radius = 0.3)
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
            mp.pub_data(map3d.get_points(), map3d.get_traversability())
            mp.pub_data(np.array(map3d.raw_points),None,'raw')
            time.sleep(0.1)






