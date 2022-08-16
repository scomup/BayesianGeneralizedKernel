#!/usr/bin/python3
# coding: UTF-8
 
import numpy as np
import matplotlib.pyplot as plt
from math import *
import random
random.seed(0)

sigma = 0.05

def get_test_data(sigma):
    x = random.uniform(0, 2*np.pi)
    y = np.cos(x)
    #if(x > 3):
    #    y = 0.
    y_prime = random.normalvariate(y, sigma)
    return x, y, y_prime
def step_signal(sigma):
    b = random.uniform(0, 1)
    b = b > 0.95
    if(b == True):
        x = 3
        y = random.uniform(-1,1)
    else:
        x = random.uniform(0, 2*np.pi)
        if(x < 3):
            y = np.cos(x)
        else:
            y = 1
    x_prime = random.normalvariate(x, sigma/2)
    y_prime = random.normalvariate(y, sigma)
    return x_prime, y, y_prime

def step_signal_true():
    x = np.arange(-2,5,0.01)
    y = np.where(x<3,np.cos(x),1)

    return x, y



class Map2D():
    def __init__(self, x_min, x_max, resolution, radius=0.2, use_b = False):
        self.resolution = resolution
        self.radius = radius
        self.location = np.arange(x_min, x_max+1, resolution)
        self.elevation = np.full_like(self.location,0) 
        self.lambdas = np.full_like(self.location,0)
        self.variance = np.full_like(self.location,1)

        self.traversability = np.full_like(self.location,0)
        self.alpha = np.full_like(self.location,0)
        self.beta = np.full_like(self.location,0)
        self.use_b = use_b

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
        k = self.sparse_kernel(x,x_star,l=self.radius)
        plt.plot(x_star,self.sparse_kernel(x,x_star,l=0.1))
        #plt.plot(x_star-x,self.sparse_kernel(x,x_star,l=0.2))
        #plt.plot(x_star-x,self.sparse_kernel(x,x_star,l=0.5))

        plt.show()
        exit()
        lam = self.lambdas[lb:ub]
        mu_0 = self.elevation[lb:ub] 
        var = self.variance[lb:ub]
        if(self.use_b):
            std = np.sqrt(self.variance[ int(x/self.resolution)-1])
            s = np.exp(-4*std)
            k = self.sparse_kernel(x,x_star, l=s*self.radius)
            #b = self.sparse_kernel(y - mu_0, 0.3)
            #b[lam < 1] = 1.
            #k=k * b
            k

        
        #w = np.abs(mu_0 - y)
        #w = np.exp(-w*10)
        #k=k*w
        y_star = (lam * mu_0 + k * y)/(lam + k)
        v_star = (lam/(lam + k))*(var + k*(mu_0 - y)**2/(lam + k))
        self.variance[lb:ub] = v_star

        self.elevation[lb:ub] = y_star
        self.lambdas[lb:ub] = lam + k

        #v = self.calc_traversability(self.location[lb:ub], self.elevation[lb:ub])
        #alpha_star = self.alpha[lb:ub] + k*v
        #beta_star = self.beta[lb:ub] + k*(1-v)
        #self.traversability[lb:ub] = alpha_star/(alpha_star + beta_star)
        #self.alpha[lb:ub] = alpha_star
        #self.beta[lb:ub] = beta_star


map2d = Map2D(0,np.pi*2,0.005,0.25)
map2db = Map2D(0,np.pi*2,0.005,0.25, use_b=True)


x_ = []
y_ = []
y_p_ = []
show = 0

fig, ax = plt.subplots(2,1)
ax[0].set_ylim(-2,2)
ax[0].set_xlim(1, 5)
ax[1].set_ylim(0,0.8)
ax[1].set_xlim(1, 5)

ax[0].plot(map2d.location,map2d.elevation,c='r', label='using fixed kernel')
ax[0].plot(map2db.location,map2db.elevation,c='b', label='using adaptive kernel')
x, y= step_signal_true()
ax[0].plot(x, y,c='g', linestyle="dotted", label='ground true')
ax[0].legend(loc='lower right', borderaxespad=1,fontsize=10)
var = map2d.variance
std = np.sqrt(var)
ax[1].plot(map2d.location, np.ones_like(map2d.location)*0.5,c='r',linestyle="dotted", label="Fixed kernel size")
ax[1].plot(map2d.location, np.exp(-2*std)*0.5,c='b',linestyle="dotted",label="Adaptive kernel size")
ax[1].legend(loc='lower right', borderaxespad=1,fontsize=10)
#plt.pause(10)

for i in range(200):
    ax[0].cla()
    ax[1].cla()

    ax[0].set_ylim(-2,2)
    ax[0].set_xlim(1, 5)
    ax[1].set_ylim(0,0.8)
    ax[1].set_xlim(1, 5)
    x, y, yp = step_signal(sigma)
    x_.append(x)
    y_.append(y)
    y_p_.append(yp)
    map2d.update(x, yp)
    map2db.update(x, yp)

var = map2d.variance
std = np.sqrt(var)
ax[0].plot(map2d.location,map2d.elevation,c='r', label='using fixed kernel')
ax[0].plot(map2db.location,map2db.elevation,c='b', label='using adaptive kernel')
x, y= step_signal_true()
ax[0].plot(x, y,c='g', linestyle="dotted", label='ground true')
ax[0].scatter(x_,y_p_,s=10,c='g', label='observed points')
ax[0].legend(loc='lower right', borderaxespad=1,fontsize=10)
ax[1].plot(map2d.location, np.ones_like(map2d.location)*0.5,c='r',linestyle="dotted", label="Fixed kernel size")
ax[1].plot(map2d.location, np.exp(-2*std)*0.5,c='b',linestyle="dotted",label="Adaptive kernel size")
ax[1].legend(loc='lower right', borderaxespad=1,fontsize=10)
plt.show()


