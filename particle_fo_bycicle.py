# -*- coding: utf-8 -*-
"""
Created on Sun Jul 09 03:24:52 2017
Sequelcial Monte Carlo
@author: Eduardo
"""

import math
import random
import matplotlib.pyplot as plt
from matplotlib import style
import sys

style.use("fivethirtyeight")
                #y    #x
landmarks = [[0.0,100.0],
             [0.0,0.0],
             [100.0,0.0],
             [100.0,100.0]]

world_size = 100.0
max_steering_angle = math.pi/4 #limitations of a real car
tolerance_xy = 15.0
tolerance_orientation = 0.25
class robot:
    #puedes crear un robot en dos dimensiones en un mundo de 100x100
    def __init__(self,length=10.0):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * math.pi
        self.length = length
        self.bearing_noise = 0.0
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        
    def setter(self,new_x,new_y,new_orientation):
        #if new_x < 0 or new_x >= world_size:
        #    raise ValueError, "X coordinate out of bound"
        #if new_y < 0 or new_y >= world_size:
        #    raise ValueError, "Y coordinate out of bound"
        if new_orientation < 0 or new_orientation >=2.0*math.pi:
            print("Orientation must be in [0.,2pi]")
            raise ValueError
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
        
    def set_noise(self,new_b_noise,new_s_noise,new_d_noise):
        self.bearing_noise = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        
    def sense(self,bearing_noise=1):
        Z = []
        #la medida se obtiene tras aplicar una gausiana
        if bearing_noise ==1: 
            for i in range(len(landmarks)):
                noise_landmarkx = random.gauss(landmarks[i][1],self.bearing_noise)
                noise_landmarky = random.gauss(landmarks[i][0],self.bearing_noise)
                delta_x = noise_landmarkx - self.x
                delta_y = noise_landmarky - self.y
                #print(delta_x,delta_y)
                bearing = math.atan2(delta_y,delta_x)
                if bearing < 0:
                    bearing = 2.0 * math.pi - abs(bearing)
                    bearing= bearing - self.orientation
                else:
                    #bearing = bearing* (180.0/math.pi)
                    bearing= bearing - self.orientation
                Z.append(bearing)
        elif bearing_noise == 0:
            for i in range(len(landmarks)):
                delta_x = landmarks[i][1] - self.x
                delta_y = landmarks[i][0] - self.y
                #print(delta_x,delta_y)
                bearing = math.atan2(delta_y,delta_x)
                if bearing < 0:
                    bearing = 2.0 * math.pi - abs(bearing)
                    bearing= bearing - self.orientation
                else:
                    #bearing = bearing* (180.0/math.pi)
                    bearing= bearing - self.orientation
                Z.append(bearing)
        return Z
    
    def car_movement(self,motions,tolerance=0.001):
        angle = motions[0]
        distance = motions[1]
        tetha = self.orientation
        #agragaondo gausian noise
        angle2 = random.gauss(angle,self.steering_noise)
        distance2 =random.gauss(distance, self.distance_noise)
        
        if abs(angle) > max_steering_angle:
            print("You overshoot the car range of angle ")
            raise ValueError
        if distance < 0.0:
            print("The car cannot go backwards")
            raise ValueError
        #turning angle
        beta = (float(distance2)/self.length)*math.tan(angle2)
        if abs(beta) < tolerance:
            new_x = self.x + float(distance2) * math.cos(tetha)
            new_y = self.y + float(distance2) * math.sin(tetha)
            new_orientation = (tetha + beta) % (2.0*math.pi)
        else:
            radius = float(distance2)/beta    
            cx = self.x - math.sin(tetha) * radius
            cy = self.y + math.cos(tetha) * radius
            #pocisiones globales
            new_x = cx + math.sin(tetha + beta) * radius
            new_y = cy - math.cos(tetha + beta) * radius
            new_orientation = (tetha + beta) % (2.0 * math.pi)
        res = robot(self.length)
        res.setter(new_x,new_y,new_orientation)
        res.set_noise(self.bearing_noise,self.steering_noise,self.distance_noise)
        return res
        
    
    def Gaussian(self,error_b,sigma):
        #calculates the probability of x for 1-dim Gaussian with mean mu
        return (1.0/math.sqrt(2.0*math.pi*sigma**2))*math.exp(-0.5*(((error_b)**2)/sigma**2))
    
    def measurement_prob(self, measurements):
        #calculates how likely a measure should be
        #which is an essential step
        predicted_measurements = self.sense(0)
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            #because this give me a value between pi,-pi we must apply %mod
            error_bearing = (error_bearing + math.pi) % (2.0 * math.pi) - math.pi #trucate
            
            #update gaussian
            #error = (math.exp(-(error_bearing**2)/(self.bearing_noise**2)/2.0)/
            #         math.sqrt(2.0*math.pi*(self.bearing_noise**2)))
            error *= self.Gaussian(error_bearing,self.bearing_noise)
        return error
    
    def __str__(self):
        return "[x=%s,y=%s,tetha=%s]"% (str(self.x),str(self.y),str(self.orientation))
    
class particles(robot):

    def __init__ (self,amount=1000):
        self.p = self.creator(amount)
        self.amount = amount
        self.color = "r"
        
    def actualize(self,turn,forward):
        count = 0
        x =int(self.amount/5)
        for i in range(x):
            self.p[count] = self.p[count].car_movement([turn,forward])
            self.p[count+1] = self.p[count+1].car_movement([turn,forward])
            self.p[count+2] = self.p[count+2].car_movement([turn,forward])
            self.p[count+3] = self.p[count+3].car_movement([turn,forward])
            self.p[count+4] = self.p[count+4].car_movement([turn,forward])
            count += 5
            
    def creator(self,amount):
        p = []
        for i in range(amount):
            xi = robot()
            p.append(xi)
        return p
    
    def setter_noise_part(self,bearing,steering,distance):
        for i in range(self.amount):
            self.p[i].set_noise(bearing,steering,distance)
            
    def importance_weight(self,Z):
        #first part of particle filter
        w = []
        for i in range(self.amount):
            w.append(self.p[i].measurement_prob(Z))
        return w
    
    def resampling(self,w):
        
        #2nd part of the particle filter
        p3 = []
        #resampling
        index = int(random.random() * self.amount)
        beta = 0.0
        mw = max(w)
        #esta parte, el resampling localiza la posicion con una desviacion estandar menor a 4.5
        for i in range(self.amount):
            beta += random.random() * 2.0 * mw
            while beta> w[index]:
                beta -= w[index]
                index = (index + 1) % self.amount
            p3.append([self.p[index].x,self.p[index].y,self.p[index].orientation])
        #print p3
        for i in range(len(p3)):
            self.p[i].x = p3[i][0]
            self.p[i].y = p3[i][1]
            self.p[i].orientation = p3[i][2]
        return True
    
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
        dotx.append(landmarks[i][1])
        doty.append(landmarks[i][0])
    plt.scatter(dotx,doty,s=200,color='k',label="obstaculos")    
    if particulas is not 0:
        [plt.scatter(particulas.p[i].x,particulas.p[i].y,s=1,color=particulas.color) for i in range(particulas.amount)]
    plt.ylim(-20,world_size+20)
    plt.xlim(-20,world_size+20)
    plt.ylabel("y")
    plt.xlabel("x")
    plt.legend(bbox_to_anchor=(1.05,1),loc=2,borderaxespad=0.)
    plt.show()

def get_position(particles):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(particles.amount):
        x+= particles.p[i].x
        y += particles.p[i].y
        #orientation is tricky because it is cyclic. By normalizing
        #around the first particle we are somewhat more robust to
        #the 0=2pi problem
        orientation += (((particles.p[i].orientation - particles.p[0].orientation + math.pi)%(2.0*math.pi))+ particles.p[0].orientation - math.pi)
    return [x/particles.amount,y/particles.amount,orientation/particles.amount]
                       
def generate_ground_truth(motions):
    #measurements vector
    vehicle = robot()
    vehicle.set_noise(bearing_noise,steering_noise,distance_noise)
    Z = []
    xs = []
    ys = []
    T = len(motions)
    plt.scatter([vehicle.x],[vehicle.y],color="y",s=200,label="robot start")
    c = ["r","g","b","m","c"]
    for t in range(T):
        vehicle = vehicle.car_movement(motions[t])
        xs.append(vehicle.x)
        ys.append(vehicle.y)
        Z.append(vehicle.sense())
    [plt.scatter(xs[i],ys[i],color=c[-i],s=200,label="robot place:{}".format(i+1)) for i in range(T)]
    return [vehicle, Z]
    
def print_measurements(Z):
    T = len(Z)
    print("measurements = [[%s,%s,%s,%s],"%(str(Z[0][0]),str(Z[0][1]),str(Z[0][2]),str(Z[0][3])))
    for t in range(1,T-1):
        print("[%s,%s,%s,%s]," %(str(Z[t][0]),str(Z[t][1]),str(Z[t][2]),str(Z[t][3])))
    print("[%s,%s,%s,%s]]"%(str(Z[T-1][0]),str(Z[T-1][1]),str(Z[T-1][2]),str(Z[T-1][3])))
        
def check_output(final_robot,estimated_position):
    error_x = abs(final_robot.x -estimated_position[0])
    error_y = abs(final_robot.y -estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + math.pi) % (2.0 * math.pi) - math.pi
    correct = error_x<tolerance_xy and error_y <tolerance_xy and error_orientation<tolerance_orientation
    return correct

def particle_filter(motions,measurements,N=500):
    #PARTICLE FILTER
    #make particles
    try:
        particle = particles(N)
        particle.setter_noise_part(bearing_noise,steering_noise,distance_noise)
        
        for i in range(len(measurements)):
            #Motion update(prediction)
            particle.actualize(motions[i][0],motions[i][1])
            
            #Measurement update
            #assigment importance weight to particle from gaussian
            w = particle.importance_weight(measurements[i])
            #print w
            particle.resampling(w)
            
        graph_simulator(myrobot,particle)
        return get_position(particle)
    except Exception as e:
        print(sys.exc_info()[0],sys.exc_info()[1],sys.exc_info()[2].tb_lineno)
        

length = 20.0
bearing_noise = 0.1
steering_noise = 0.1
distance_noise =5.0

myrobot = robot(length)
myrobot.set_noise(bearing_noise,steering_noise,distance_noise)
myrobot.setter(0.0,0.0,0.0)

"""test 1"""
print("TEST 1")
motions_0=[[0.0,10.0],[math.pi/6.0,10.0],[0.0,20.0]]
motions_2 = [[0.2,10.0] for row in range(10)]
T = len(motions_2)
print("Robot start: ",myrobot)

for t in range(T):
    myrobot = myrobot.car_movement(motions_2[t])
    print("Robot: ", myrobot)
    
"""test 2"""
print("TEST 2")
myrobot.setter(30.0,20.0,0.0)
print("robot: ",myrobot)
print("Measurement: ",myrobot.sense())

"""test 3"""
print("TEST 3")
motions_1 = [[2.0*math.pi/10.0,20.0] for row in range(8)]
measurements_1 = [[4.746936, 3.859782, 3.045217, 2.045506],
                [3.510067, 2.916300, 2.146394, 1.598332],
                [2.972469, 2.407489, 1.588474, 1.611094],
                [1.906178, 1.193329, 0.619356, 0.807930],
                [1.352825, 0.662233, 0.144927, 0.799090],
                [0.856150, 0.214590, 5.651497, 1.062401],
                [0.194460, 5.660382, 4.761072, 2.471682],
                [5.717342, 4.736780, 3.909599, 2.342536]]

print(particle_filter(motions_1, measurements_1))

"""test 4"""
print("TEST 4")
number_of_iterations = 6
motions = [[2. * math.pi / 20, 12.] for row in range(number_of_iterations)]
##
x = generate_ground_truth(motions)
final_robot = x[0]
measurements = x[1]
estimated_position = particle_filter(motions, measurements)
print_measurements(measurements)
print('Ground truth:    ', final_robot)
print('Particle filter: ', estimated_position)
print('Code check:      ', check_output(final_robot, estimated_position))
input()