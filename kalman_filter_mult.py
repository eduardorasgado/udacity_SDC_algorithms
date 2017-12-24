# -*- coding: utf-8 -*-
"""
Created on Wed Jul 05 15:08:14 2017
Filtro de Kalman multidimensional
@author: Eduardo
"""
from math import * 

class matrix:
    #implementa las operaciones básicas de una clase matriz
    def __init__(self,value):
        self.value = value
        self.dimx = len(value)  #x como filas
        self.dimy = len(value[0])  #y como las columnas
        if self.value == [[]]:
            self.dimx = 0
    
    def zero(self,dimx,dimy):
        #checa la validez de las dimensiones para crear una matriz de ceros
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for col in range(dimy)] for row in range(dimx)]
            
    def identity(self,dim):
        #Verifica la validez de la dimension de entrada de la matriz identidad
        if dim < 1:
            raise ValueError,"Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for col in range(dim)] for row in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
     
    def show(self):
        #muestra el arreglo final
        if self.dimx == 1:
            print(self.value)
            return True
        for i in range(self.dimx):
            if i==0:
                print("[{},\n".format(self.value[i]))
            if i == self.dimx-1:
                print("{}]\n".format(self.value[i]))
            elif i != 0 and i != self.dimx-1:
                print("{},\n".format(self.value[i]))
        print(' ')        
        
    def __add__(self,other):
        #checamos las dimensiones sean correctas
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            res = matrix([[]])
            res.zero(self.dimx,self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
  
    def __sub__(self,other):
        #verificamos si estan las dimensionescorrectas
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to substract"
        else:
            res = matrix([[]])
            res.zero(self.dimx,self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self,other):
        
        if self.dimy != other.dimx: #si las columnas del primero no son iguales a las filas de la otra matriz
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            res = matrix([[]])
            res.zero(self.dimx,other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
        
    def transpose(self):
        res = matrix([[]])
        res.zero(self.dimy,self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    def Cholesky(self,ztol=1.0e-5):
        #determina la upper triangula factorizacion de Cholesky
        #de una matriz positiva definida
        res = matrix([[]])
        res.zero(self.dimx,self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1,self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res

    def CholeskyInverse(self):
        #Inversa de la matriz dada su Cholesky upper Triangular
        #descomposicion de matriz
        res = matrix([[]])
        res.zero(self.dimx,self.dimx)
        
        #pasos atras para invertir
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k] * res.value[j][k] for k in range(j+1,self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1,self.dimx)])/self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


#Implementacion del filtro de Kalman
def kalman_filter(x, P):
    for n in range(len(measurements)):
        # prediction
        x = F * x + u
        P = F *  P * F.transpose()
        
        # measurement update
        Z= matrix([measurements[n]])
        #z.transpose hace que sea mas facil trabajar con medidas de varias dimensiones
        y = Z.transpose() -(H*x)  #error de calculo
        S = H * P * H.transpose() + R  
        K = (P * H.transpose()) * S.inverse()  #ganancia de kalman
        x = x + (K * y)
        P = (I - (K * H)) * P
        
    print("prediccion: ") 
    x.show()
    print("inferencia(covarianza): ") 
    P.show()
    
    return x,P

"""
measurements = [1.,2.,3.] #1D pero con inferncia sobre la velocidad

x = matrix([[0.],[0.]])  #estado inicial(lugar y velocidad)
P = matrix([[1000.,0.],[0.,1000.]])  #incertidumbre inicial
u = matrix([[0.],[0.]])  #movimiento externo
F = matrix([[1.,1.],[0.,1.]]) #funcion del siguiente estado
H = matrix([[1.,0.]])  #funcion de medicion
R = matrix([[1.]])  #medicion incertidumbre, ruido
I = matrix([[]])
I.identity(2)  #matriz identidad

print kalman_filter(x,P)
"""

#Ahora entramos a ejercicios en dos dimensiones

measurements = [[5.,10.],[6.,8.],[7.,6.],[8.,4.],[9.,2.],[10.,0.]] #2D con inferencias sobre dos velocidades
initial_xy = [4.,12.]
measurements_1 = [[1.,17.],[1.,15.],[1.,13],[1.,11]]
dt = 0.1
x = matrix([[initial_xy[0]],
            [initial_xy[1]],
            [0.],
            [0.]])  #estado inicial(lugar y velocidad)
                    
u = matrix([[0.],
            [0.],
            [0.],
            [0]])  #movimiento externo

 # 0 es la incertidumbre sobre la localizacion y 1000 es para la veocidad
P = matrix([[0.,0.,0.,0.],
            [0.,0.,0.,0.],
            [0.,0.,1000.,0.],
            [0.,0.,0.,1000.]])  #incertidumbre inicial
                    
F = matrix([[1.,0.,dt,0.],
            [0.,1.,0.,dt],
            [0.,0.,1.,0.],
            [0.,0.,0.,1.]]) #funcion del siguiente estado

#H refleja el efecto de no poder ver las dos segundas dimensiones: x y velocidad
H = matrix([[1.,0.,0.,0.],
            [0.,1.,0.,0.]])  #funcion de medicion
                    
R = matrix([[0.1,0.],
            [0.,0.1]])  #medicion incertidumbre, ruido
                    
I = matrix([[]])
I.identity(4)  #matriz identidad

print kalman_filter(x,P)

