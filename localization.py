# -*- coding: utf-8 -*-
"""
Created on Mon Jul 03 19:04:32 2017
Monte Carlo Robot localization in 2D

Method = Histogram Methods
@author: Admin
"""
colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]

color_test1 = [['G','G','G'],
               ['G','R','G'],
               ['G','G','G']]
def sense(p,colors,measurement):
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]
    s = 0.0
    #no normalizado
    for i in range(len(p)):
        for j in range(len(p[i])):
            hit = (measurement == colors[i][j])
            aux[i][j] = p[i][j] * (hit * sensor_right + (1-hit)* sensor_wrong)
            s += aux[i][j]
    #print(aux)
    
    #normalizado        
    for i in range(len(aux)):
        for j in range(len(p[i])):
            aux[i][j] /=s
    return aux

def move(p,motion):
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]
    
    for i in range(len(p)):
        for j in range(len(p[i])):
            aux[i][j] = p_move * p[(i-motion[0])%len(p)][(j-motion[1])%len(p[i])] + (p_stay * p[i][j])
    return aux

def core_localizer(colors,measurements,motions,sensor_right,p_move):
    #inicializa p a una distribucion de una malla de las mismas
    #dimensiones que los colores
    if len(measurements)!= len(motions):
        raise ValueError, "error en tamaño de vector de medida/movimiento" 
    
    pinit = 1.0/float(len(colors))/float(len(colors[0]))
    #uno sobre las columnas sobre las filas
    p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]
    for k in range(len(measurements)):
        p= move(p,motions[k])
        p = sense(p,colors,measurements[k])             
    return p

def showing(p):
    #muestra el arreglo final
    for i in range(len(p)):
        if i==0:
            print("[{},\n".format(p[i]))
        if i == len(p)-1:
            print("{}]\n".format(p[i]))
        elif i != 0 and i != len(p)-1:
            print("{},\n".format(p[i]))
    
st_measure = ['R']
motion_test = [[0,0]]
measurements = ['G','G','G','G','G']
"""
motions:
    [0,0] = stay
    [0,1] = right
    [0,-1] = left
    [1,0] = down
    [-1,0] = up
"""
#podemos modificar los parametros de move y sensor_right a 1.0
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]
sensor_right = 0.7
p_move = 0.8
sensor_wrong = 1.0 - sensor_right
p_stay = 1.0 - p_move

p = core_localizer(colors,measurements,motions,sensor_right,p_move)
showing(p)



