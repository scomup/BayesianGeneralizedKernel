#!/usr/bin/python3
# coding: UTF-8
 
import numpy as np
import matplotlib.pyplot as plt
from math import *
import random


sigma = 0.1

def get_test_data(sigma):
    x = random.uniform(0, 2*np.pi)
    y = np.cos(x)
    if(x > 3):
        y = 0.
    y_prime = random.normalvariate(y, sigma)
    return x, y_prime

class Map2D():
    def __init__(self, x_min, x_max, resolution, radius=0.2):
        self.resolution = resolution
        self.radius = radius
        self.location = np.arange(x_min, x_max, resolution)
        self.elevation = np.full_like(self.location,0) 
        self.lambdas = np.full_like(self.location,0)

    def sparse_kernel(self, x, x_star,l=0.5):
        d = np.abs(x_star-x)
        r = d/l
        pi = np.pi
        k = (2 + np.cos(2*pi*r))/3*(1-r) + np.sin(2*np.pi*r)/(2*np.pi)
        k[d>=l] = np.finfo(float).eps
        return k

    def update(self, x, y):
        lb = int(np.clip( (x - self.radius)/self.resolution, 0, np.size(self.location)))
        ub = int(np.clip( (x + self.radius)/self.resolution, 0, np.size(self.location)))
        x_star = self.location[lb:ub] #x_star represents the map locations that are within distance radius
        k = self.sparse_kernel(x,x_star)
        lam = self.lambdas[lb:ub]
        mu_0 = self.elevation[lb:ub] 
        y_star = (lam * mu_0 + k * y)/(lam + k)
        self.elevation[lb:ub] = y_star
        self.lambdas[lb:ub] = lam + k
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
show = True
for i in range(10000):
    x, y = get_test_data(sigma)
    x_.append(x)
    y_.append(y)
    map2d.update(x, y)
    if(show == True):
        var = sigma**2/map2d.lambdas
        std = np.sqrt(var)
        plt.scatter(x, y, c='r')
        plt.plot(map2d.location,map2d.elevation, label='predicted points')
        plt.plot(map2d.location,np.cos(map2d.location), label='real points',linestyle="dashed")
        plt.plot(map2d.location,map2d.elevation + 3*std,c='b',linestyle="dotted")
        plt.plot(map2d.location,map2d.elevation - 3*std,c='b',linestyle="dotted", label='3 sigma')
        #plt.plot(map2d.location,map2d.lambdas)
        plt.scatter(x_,y_,s=3,c='g', label='observed points')
        plt.legend(loc='lower right', borderaxespad=1)

        plt.pause(0.1)
        plt.cla()
        plt.ylim(-1.5, 1.5)
        plt.xlim(0, 2*np.pi)

