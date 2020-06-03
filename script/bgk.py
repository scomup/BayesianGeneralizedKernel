#!/usr/bin/python3
# coding: UTF-8
 
import numpy as np
import matplotlib.pyplot as plt
from math import *
import random


sigma = 0.01

def get_test_data(sigma):
    x = random.uniform(0, 2*np.pi)
    y = np.cos(x)
    if(x > 3):
        y = 0.
    y_prime = random.normalvariate(y, sigma)
    return x, y_prime
def step_signal(sigma):
    b = random.uniform(0, 1)
    b = b > 0.95
    if(b == True):
        x = 3
        y = random.uniform(0, 0.5)
    else:
        x = random.uniform(0, 2*np.pi)
        if(x < 3):
            y = 0
        else:
            y = 0.5
    x_prime = random.normalvariate(x, sigma)
    y_prime = random.normalvariate(y, sigma)
    return x_prime, y_prime
class Map2D():
    def __init__(self, x_min, x_max, resolution, radius=0.2):
        self.resolution = resolution
        self.radius = radius
        self.location = np.arange(x_min, x_max, resolution)
        self.elevation = np.full_like(self.location,0) 
        self.lambdas = np.full_like(self.location,0)
        self.variance = np.full_like(self.location,1)

        self.traversability = np.full_like(self.location,0)
        self.alpha = np.full_like(self.location,0)
        self.beta = np.full_like(self.location,0)

    def sparse_kernel(self, x, x_star,l=0.5):
        d = np.abs(x_star-x)
        r = d/l
        pi = np.pi
        k = (2 + np.cos(2*pi*r))/3*(1-r) + np.sin(2*np.pi*r)/(2*np.pi)
        k[d>=l] = np.finfo(float).eps
        return k

    def calc_traversability(self, location, elevation):
        points = np.stack([location, elevation])
        center = np.mean(points, axis=1)
        max_step = np.max(points[1,:]) - np.min(points[1,:])
        cov = np.cov(points, rowvar=1)
        w, v = np.linalg.eig(cov)
        slope = np.arccos(np.abs(v[1,1]))/ np.pi * 180
        if(np.isnan(slope)):
            print(slope)

        h = 1. / (1. + np.exp(-(slope - 30.0)))
        s = max_step / 2.0
        r = np.sqrt(cov[1,1] )/ 3.0
        v = 0
        if (h > 1 or s > 1 or r > 1):
            v = 1
        else:
            v = np.min([0.9 * h + 0.105 * s + 0.05 * r, 1.0])
        return v


    def update(self, x, y):
        lb = int(np.clip( (x - self.radius)/self.resolution, 0, np.size(self.location)))
        ub = int(np.clip( (x + self.radius)/self.resolution, 0, np.size(self.location)))
        x_star = self.location[lb:ub] #x_star represents the map locations that are within distance radius
        k = self.sparse_kernel(x,x_star)
        plt.plot(x_star-x,self.sparse_kernel(x,x_star,l=0.1))
        plt.plot(x_star-x,self.sparse_kernel(x,x_star,l=0.2))
        plt.plot(x_star-x,self.sparse_kernel(x,x_star,l=0.5))

        plt.show()
        lam = self.lambdas[lb:ub]
        mu_0 = self.elevation[lb:ub] 
        var = self.variance[lb:ub]
        
        #w = np.abs(mu_0 - y)
        #w = np.exp(-w*10)
        #k=k*w
        y_star = (lam * mu_0 + k * y)/(lam + k)
        v_star = (lam/(lam + k))*(var + k*(mu_0 - y)**2/(lam + k))
        self.variance[lb:ub] = v_star

        self.elevation[lb:ub] = y_star
        self.lambdas[lb:ub] = lam + k

        v = self.calc_traversability(self.location[lb:ub], self.elevation[lb:ub])
        alpha_star = self.alpha[lb:ub] + k*v
        beta_star = self.beta[lb:ub] + k*(1-v)
        self.traversability[lb:ub] = alpha_star/(alpha_star + beta_star)
        self.alpha[lb:ub] = alpha_star
        self.beta[lb:ub] = beta_star

        #if(self.show == True):
        #    var = sigma**2/self.lambdas
        #    std = np.sqrt(var)
        #    plt.scatter(x, y, c='r')
        #    plt.plot(location,elevation)
        #    plt.plot(location,elevation + 3*std,c='b',linestyle="dotted")
        #    plt.plot(location,elevation - 3*std,c='b',linestyle="dotted")
        #    plt.scatter(x_,y_,s=3,c='g')
        #    plt.pause(0.1)
        #    plt.cla()
        #    plt.ylim(-1.5, 1.5)
        #    plt.xlim(0, 2*np.pi)


map2d = Map2D(0,np.pi*2,0.01,0.4)


plt.ylim(-3, 3)
plt.xlim(0, 2*np.pi)

x_ = []
y_ = []
show = 0
for i in range(10000):
    x, y = step_signal(sigma)
    x_.append(x)
    y_.append(y)
    map2d.update(x, y)
    show += 1
    if(show == 100):
        show = 0
        var = map2d.variance
        std = np.sqrt(var)
        plt.scatter(x, y, c='r')
        plt.plot(map2d.location,map2d.elevation, label='predicted points')
        plt.plot( map2d.location,map2d.variance*10, label='variance',linestyle="dashed")
        s = 1
        plt.plot(map2d.location,map2d.elevation + s*std,c='b',linestyle="dotted")
        plt.plot(map2d.location,map2d.elevation - s*std,c='b',linestyle="dotted", label='3 sigma')
        #plt.plot(map2d.location,map2d.lambdas)
        plt.scatter(x_,y_,s=3,c='g', label='observed points')
        plt.legend(loc='lower right', borderaxespad=1)

        plt.pause(0.1)
        plt.waitforbuttonpress()
        plt.cla()
        plt.ylim(-1.5, 1.5)
        plt.xlim(0, 2*np.pi)

