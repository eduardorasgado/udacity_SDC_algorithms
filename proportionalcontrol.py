# -*- coding: utf-8 -*-
"""
Created on Sun Jul 16 10:57:18 2017
P control
@author: Eduardo Rasgado
"""


#the desire trajectory for the robot is the x axis
#stering angle = -tau * crosstrack_error

import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style


style.use("ggplot")

class robot(object):
    def __init__(self,length=20.0):
        #xcreates robot and initializes location/orientaton to 0,0,0.
        self.x = 0.0
        self.y = 0.0
        self.orientation= 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0
        
    def setter(self,x,y,orientation):
        #sets a robot coordinate
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)
        
    def set_noise(self, steering_noise,distance_noise):
        #set the noise parameters
        #this is ften useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise
        
    def set_steeringdrift(self,drift):
        #sets the systematical steering drift parameter
        self.steering_drift = drift
        
    def move(self, steering, distance,tolerance=0.001,max_steering_angle=np.pi/4.0):
        #steering = front wheel steering angle, limited by max_steering_angle
        #distance = total distance driven, most be non-negative
        if steering > max_steering_angle:
            steering= max_steering_angle
        if steering < -max_steering_angle:
            steering=-max_steering_angle
        if distance < 0.0:
            distance = 0.0
        #make a new copy
        res = robot()
        res.length = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.steering_drift = self.steering_drift
        
        #appying noise
        steering2 = random.gauss(steering,self.steering_noise)
        distance2 = random.gauss(distance,self.distance_noise)
        
        #apply steering_drift
        steering2 += self.steering_drift
        
        #Execute motion
        #beta = tan(alpha)x d/L
        turn = np.tan(steering2) * distance2 / self.length
        
        if abs(turn) <tolerance:  #linea recta
            res.x = self.x + distance2 * np.cos(self.orientation)
            res.y = self.y + distance2 * np.sin(self.orientation)
            res.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            #turn > 0
            #approximate bycicle model for motion
            #globales:                       
            #R = d/beta
            radius = distance2 / turn
            #cx = x-Rsin(tetha)
            cx = self.x - (radius*np.sin(self.orientation))
            #cy = y+Rcos(tetha)
            cy = self.y + (radius*np.cos(self.orientation))
            #tetha = (tetha+betha)mod 2pi
            res.orientation = (self.orientation + turn) % (2.0*np.pi)
            #x = cx+Rsin(tetha+beta)
            res.x = cx + (np.sin(res.orientation)*radius)
            #y = cy - Rcos(tetha+betha)
            res.y = cy - (np.cos(res.orientation)*radius)
            
        return res
    
    def __repr__(self):
        return '[x=%s, y=%s, orient=%s]'%(self.x,self.y,self.orientation)
    
def run_proportional(param):
    myrobot = robot()
    myrobot.setter(0.0,1.0,0.0)
    #myrobot.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
    speed = 1.0 #motion distance equalt to speed (we assume time = 1)
    N=500
    tau =-param
    xs = []
    ys = []
    for i in range(N): 
        xs.append(myrobot.x)
        ys.append(myrobot.y)
        a = myrobot.y*tau
        steer = a
        myrobot = myrobot.move(steer,speed)  
        if i==150:
            myrobot.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
        
        #print(myrobot,steer)
    return xs,ys

def run_proportional_derivative(param1,param2):
    myrobot2 = robot()
    myrobot2.setter(0.0,1.0,0.0)
    #myrobot2.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
    speed = 1.0 #motion distance equalt to speed (we assume time = 1)
    N=500
    dt =1.0
    tau_p =param1
    tau_d =param2
    xs2 = []
    ys2 = []
    last_error = myrobot2.y
    for i in range(N):     
        diff_cte = myrobot2.y - last_error
        steering_angle = -tau_p * myrobot2.y - tau_d * (diff_cte/dt)
        last_error = myrobot2.y
        myrobot2 = myrobot2.move(steering_angle,speed)
        if i==150:
            myrobot2.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
        xs2.append(myrobot2.x)
        ys2.append(myrobot2.y)
        #print(myrobot2,steering_angle)
    return xs2,ys2

def run_proportional_integrative_derivative(param1,param2,param3):
    myrobot = robot()
    myrobot.setter(0.0,1.0,0.0)
    #myrobot.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
    speed = 1.0 #motion distance equalt to speed (we assume time = 1)
    N=500
    dt =1.0
    tau_p =param1
    tau_d =param2
    tau_i = param3
    xs = []
    ys = []
    last_error = myrobot.y
    integrated_error =0.0
    for i in range(N):     
        diff_cte = myrobot.y - last_error
        integrated_error +=myrobot.y
        last_error = myrobot.y
        steering_angle = -tau_p * myrobot.y - tau_d * (diff_cte/dt) -tau_i * integrated_error
        if i==150:
            myrobot.set_steeringdrift(10.0/180.0*np.pi) #10 degrees
        myrobot = myrobot.move(steering_angle,speed)
        xs.append(myrobot.x)
        ys.append(myrobot.y)
        #print(myrobot,steering_angle)
    return xs,ys

def make_robot():
    #reset the robot back
    robot0 = robot()
    robot0.setter(0,1.0,0)
    robot0.set_steeringdrift(10.0/180.0*np.pi)
    return robot0

def run_test(roboto, params,n=100,speed=1.0):
    x_trajectory = []
    y_trajectory = []
    err = 0
    prev_cte = roboto.y
    int_cte = 0.0
    for i in range(2*n):
        cte = roboto.y
        diff_cte = (cte-prev_cte)/speed
        int_cte+= cte
        prev_cte = cte
        steer = -params[0]*cte -params[1]*diff_cte - params[2]*int_cte
        roboto = roboto.move(steer,speed)
        x_trajectory.append(roboto.x)
        y_trajectory.append(roboto.y)
        if i >=n:
            #suma de los errores
            err += cte**2
    return x_trajectory, y_trajectory, err/n #promedio de errores
                  
def twiddle(tol=0.2):
    p = [1.0,0.0,0.004]
    dp = [1,1,1]
    robot11 = make_robot()
    x_trajectory,y_trajectory,best_err = run_test(robot11,p)
    it = 0
    while sum(dp)>tol:
        print("Iteration {}, best error = {}".format(it,best_err))
        for i in range(len(p)):
            p[i] +=dp[i]
            robot11 = make_robot()
            x_trajectory,y_trajectory, err = run_test(robot11,p)
            
            if err<best_err:
                best_err = err
                dp[i]*=1.1
            else:
                p[i] -= 2.0 * dp[i]
                robot11 = make_robot()
                x_trajectory,y_trajectory,err = run_test(robot11,p)
                
                if err<best_err:
                    best_err = err
                    dp[i] += 1.1
                else:
                    p[i]+=dp[i]
                    dp[i]*=0.9
        it+=1
    return p

        
roadx=[i for i in range(500)]
roady =[0 for i in range(500)]


xs,ys = run_proportional(0.1) #try with 0.3
#plt.ylim(-10,10)
plt.plot(xs,ys,'r--')
plt.plot(roadx,roady,'k-',label="road")
plt.legend(loc=4)
plt.title("Control proporcional")
plt.show()

xs2,ys2= run_proportional_derivative(0.2,3.0) #try with 0.3
#plt.ylim(-10,10)
plt.plot(xs2,ys2,'b--')
plt.plot(roadx,roady,'k-',label="road")
plt.legend(loc=4)
plt.title("Control proporcional derivativo")
plt.show()

xs3,ys3= run_proportional_integrative_derivative(0.2,3.0,0.004) #try with 0.3
#plt.ylim(-10,10)
plt.plot(xs3,ys3,'m--')
plt.plot(roadx,roady,'k-',label="road")
plt.legend(loc=4)
plt.title("Control PID")
plt.show()

#ajustando parametros de kp, kd, ki
params = twiddle()
kp,kd,ki = params
robot1 = make_robot()

print(kp,kd,ki)
#testing de las constantes obtenidas
x_trajectory,y_trajectory,err = run_test(robot1,params)
xs4,ys4= run_proportional_integrative_derivative(kp,kd,ki)
plt.plot(xs4,ys4,'k--')
plt.plot(x_trajectory,y_trajectory,'c--')
plt.plot(roadx,roady,'y-',label="road")
plt.legend(loc=4)
plt.title("Control PID well tunning")
plt.show()
"""
A valores p y dp
p = [1.0,0.0,0.004]
dp = [1,1,1]
#PID OPTIMO
#Kp: 1.9012608143350915
#Kd:7.798900340998299
#Ki: 0.20879803437020628
"""