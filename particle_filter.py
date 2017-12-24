# -*- coding: utf-8 -*-
"""
Created on Thu Jul 06 02:56:18 2017
Robot class
Particle Filter or Sequencial Monte Carlo
@author: EduardoRR
"""
import math
import random
import matplotlib.pyplot as plt
from matplotlib import style

style.use("fivethirtyeight")

landmarks = [[20.0,20.0],
             [80.0,80.0],
             [20.0,80.0],
             [80.0,20.0]]

world_size = 100.0

class robot:
    #puedes crear un robot en dos dimensiones en un mundo de 100x100
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * math.pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0
        
    def setter(self,new_x,new_y,new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError, "X coordinate out fo bound"
        if new_y < 0 or new_y >= world_size:
            raise ValueError, "Y coordinate out of bound"
        if new_orientation < 0 or new_orientation >=2*math.pi:
            raise ValueError, "Orientation must be in [0.,2pi]"
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
        
    def set_noise(self,new_f_noise,new_t_noise,new_s_noise):
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)
        
    def sense(self):
        Z = []
        #la medida se obtiene tras aplicar una gausiana
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x - landmarks[i][0])**2 + (self.y -landmarks[i][1])**2)
            dist += random.gauss(0.0,self.sense_noise)
            Z.append(dist)
        return Z
    
    def move(self,turn,forward):
        if forward < 0:
            raise ValueError, "Robot cant move backwards"
            
        #turn and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0,self.turn_noise)
        orientation %= 2 * math.pi
        
        #move and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0,self.forward_noise)
        x = self.x + (math.cos(orientation)*dist)
        y = self.y + (math.sin(orientation)* dist)
        x %= world_size  #cyclic truncate
        y %= world_size
        
        #set particle
        res = robot()
        res.setter(x,y,orientation)
        res.set_noise(self.forward_noise,self.turn_noise,self.sense_noise)
        return res
    
    def Gaussian(self,mu,sigma,x):
        #calculates the probability of x for 1-dim Gaussian with mean mu
        return (1.0/math.sqrt(2.0*math.pi*sigma))*math.exp(-0.5*(((x-mu)**2)/sigma))
    
    def measurement_prob(self, measurement):
        #calculates how likely a measure should be
        #which is an essential step
        prob = 1.0
        for i in range(len(landmarks)):
            dist = math.sqrt((self.x-landmarks[i][0])**2 + (self.y-landmarks[i][1])**2)
            #we use the gaussian function above, it measure
            #how far away the predicted measurements of landmarks
            #would be from the actual measurement
            prob *= self.Gaussian(dist,self.sense_noise,measurement[i])
        return prob
    
    def __str__(self):
        highs = "Ubicacion: ", [self.x,self.y,self.orientation]
        return str(highs)
    
class particles(robot):

    def __init__ (self,amount=1000):
        self.p = self.creator(amount)
        self.amount = amount
        self.color = "r"
        
    def actualize(self,turn,forward):
        count = 0
        for i in range(self.amount/5):
            self.p[count] = self.p[count].move(turn,forward)
            self.p[count+1] = self.p[count+1].move(turn,forward)
            self.p[count+2] = self.p[count+2].move(turn,forward)
            self.p[count+3] = self.p[count+3].move(turn,forward)
            self.p[count+4] = self.p[count+4].move(turn,forward)
            count += 5
            
    def creator(self,amount):
        p = []
        for i in range(amount):
            xi = robot()
            p.append(xi)
        return p
    
    def setter_noise_part(self,forward,turn,sense):
        for i in range(self.amount):
            self.p[i].set_noise(forward,turn,sense)
            
    def importance_weight(self,Z):
        #first part of particle filter
        w = []
        for i in range(self.amount):
            w.append(self.p[i].measurement_prob(Z))
        return w
    
    def resampling(self):
        #2nd part of the particle filter
        pass
    
    def __str__(self):
        f = self.amount
        bunch = []
        for i in range(f):
            xi = [self.p[i].x,self.p[i].y,self.p[i].orientation]
            bunch.append(xi)
        return str(bunch)
            
            
def graph_simulator(myrobot,particulas):
    plt.title("Simulador de filtro de particulas en movimiento de robot, 2D")
    plt.scatter([myrobot.x],[myrobot.y],color="c",s=200,label="robot:{},{}".format(myrobot.x,myrobot.y))
    plt.plot([myrobot.x],[myrobot.y],"w+",markeredgewidth=3,markersize=15)
    dotx = []
    doty = []
    for i in range(len(landmarks)):
        dotx.append(landmarks[i][0])
        doty.append(landmarks[i][1])
    plt.scatter(dotx,doty,s=200,color='m',label="obstaculos")    
    if particulas is not 0:
        [plt.scatter(particulas.p[i].x,particulas.p[i].y,s=1,color=particulas.color) for i in range(particulas.amount)]
    plt.ylim(0,world_size)
    plt.xlim(0,world_size)
    plt.ylabel("y")
    plt.xlabel("x")
    plt.legend(bbox_to_anchor=(1.05,1),loc=2,borderaxespad=0.)
    plt.show()

def evaluate(r,p):
    suma = 0.0
    for i in range(p.amount):  #calculate mean error
        dx = (p.p[i].x -r.x + (world_size/2.0))%world_size - (world_size/2.0)
        dy = (p.p[i].y -r.y + (world_size/2.0))%world_size - (world_size/2.0)
        err = math.sqrt(dx * dx + dy * dy)
        suma += err
    return suma / float(p.amount)

myrobot = robot()
myrobot.set_noise(5.0,0.1,5.0)
myrobot.setter(30.0,50.0,math.pi/2.0)

#creando 100 particulas
particle = particles()
particle.setter_noise_part(0.05,0.05,5.0)
#print(particle)
#print(particle.p[0].x)
graph_simulator(myrobot,particle)
print(myrobot)
print("sensing...")
print(myrobot.sense())

#movamos el robot
myrobot = myrobot.move(-math.pi/2.,15.0)
#las particulas se mueven
particle.actualize(0.1,20.0)
graph_simulator(myrobot,particle)
print(myrobot)
print("sensing...")
print(myrobot.sense())

myrobot = myrobot.move(-math.pi/2.,15.0)
particle.actualize(0.1,20.0)
graph_simulator(myrobot,particle)
print(myrobot)
print("sensing...")
print(myrobot.sense())

myrobot = myrobot.move(-math.pi/2.,10.0)
particle.actualize(0.1,20.0)
graph_simulator(myrobot,particle)
print(myrobot)
print("sensing...")
print(myrobot.sense())

#PARTICLE FILTER
for i in range(10):
    #assigment importance weight to particle from gaussian
    myrobot = myrobot.move(0.5,5.0)
    Z = myrobot.sense()
    w = particle.importance_weight(Z)
    #print w
    p3 = []
    #resampling
    index = int(random.random() * particle.amount)
    beta = 0.0
    mw = max(w)
    #esta parte, el resampling localiza la posicion con una desviacion estandar menor a 4.5
    for i in range(particle.amount):
        beta += random.random() * 2.0 * mw
        while beta> w[index]:
            beta -= w[index]
            index = (index + 1) % particle.amount
        p3.append([particle.p[index].x,particle.p[index].y,particle.p[index].orientation])
    #print p3
    for i in range(len(p3)):
        particle.p[i].x = p3[i][0]
        particle.p[i].y = p3[i][1]
        particle.p[i].orientation = p3[i][2]
    print(evaluate(myrobot,particle))
    
graph_simulator(myrobot,particle)
#print(particle)
#print(particle.p[0].x)              