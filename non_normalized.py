# -*- coding: utf-8 -*-
"""
Created on Sun Jul 02 15:12:32 2017
Movimiento y senso de robot
Core program del algoritmo de localizacion de los google selfdrving
cars
Simulacion
@author: Admin
"""

p = []
world = ["green","red","red","green","green"]
Z = ["red","red"]
motions = [1,1]
n= 5
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def modify(p,n):
    for i in range(n):
        k = float(n)
        p.append(1.0/k)
    return p

def non_normalized(p,pHit,pMiss,world,Z):
    
    q = []
    new_p = []
    for i in range(len(p)):
        q.append([world[i],p[i]])
        
    for i in range(len(q)):
        if q[i][0] == Z:
            q[i][1] = q[i][1]*pHit
        else:
            q[i][1] = q[i][1]*pMiss
        new_p.append(q[i][1])
    
    sum_all_p = sum(new_p)
    return q, sum_all_p, new_p
"""
#o lo que es lo mismo
def sense(p,Z):
    d = []
    for i in range(len(p)):
        hit = (Z == world[i])
        d.append(p[i]*(hit*pHit+(1-hit)*pMiss))
    return d    
"""
#actualizacion de medicion
def normalize_p(new_p,sum_all_p): 
    try:
        for i in range(len(new_p)):
            new_p[i] = new_p[i]/sum_all_p
    except Exception as e:
        print(str(e))
    return new_p

#aplicar distribucion y probabilidad a la normalizada
def move(p,U):  #u es motion number cell a la derecha
    if U==0:
        return p      
    x = ["a","b","c","d","e","f","g","h","i","j","k","l","m",
         "n","o","p","q"]
    aux = []
    index = []
    tope = len(p)   
    for i in range(len(p)):
        if p[i] != 0:
            aux.append(p[i])
            index.append(i)
            #print("indice: ",i)
    #print(aux)
    names = [x[i] for i in range(len(aux))]
    dict_p = {}
    for i in range(len(aux)):
        dict_p[names[i]] = []
    #print(dict_p)   
    count= 0
    
    for key in dict_p:
        k = []
        for i in range(len(p)):
            k.append(p[i])
            
        for i in range(len(p)):
            if i != index[count]:
                k[i] = 0.0
            elif i == index[count]:
                k[i] = k[i]
            #print("miraaa",k)
            
        for i in range(len(k)):
            t = i
            if k[t] == aux[count]:
               k[i] = k[i]* pExact
               if t+1 >= tope:
                   hash_1 = abs(tope - (t + 1))
                   k[hash_1] = aux[count]*pOvershoot
               if t+1 <tope:
                   k[t+1] = aux[count]*pOvershoot
               k[i-1] = aux[count]*pUndershoot 
        qu =[]        
        a = [i for i in k[-U:]]
        b = [i for i in k[:-U]]
        for i in range(len(a)):
            qu.append(a[i])
        for i in range(len(b)):
            qu.append(b[i])
        dict_p[key] = qu            
        count += 1
    #print(dict_p)
    
    sumarizer = [0 for i in dict_p[names[0]]]
    for key_1 in dict_p:
        #print(dict_p[key_1])
        for num in range(len(dict_p[key_1])):
            sumarizer[num] += dict_p[key_1][num]
            
    return sumarizer
    
"""
#move basic: o lo que es lo mismo
def move1(p,U):
    q = []
    for i in range(len(p)):
        q.append(p[(i-U) % len(p)])
    return q
"""

def move1(p,U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U)%len(p)]
        s = s + pOvershoot * p[(i-U-1)%len(p)]
        s = s + pUndershoot * p[(i-U+1)%len(p)]
        q.append(s)
    return q
#funcion con normalizacion y aplicacion de probabilidad

pHit = 0.6
pMiss = 0.2
"""
for i in range(len(Z)):
    non_q, sumarized, new_p = non_normalized(p,pHit,pMiss,world,Z[i])
    print("aplicado valores de confianza: ",non_q)
    print("\n")
    #print("sumatoria no normailizada: ", sumarized)
    #print("\n")
    print("lista nueva p: ", new_p)
    print("\n")
    #print(sense(p,Z))
    j = normalize_p(new_p,sumarized)
    print(j)
    print("-------------")
"""
#modify(p,n)
#print(p)
#print("\n")
#non_q, sumarized, new_p = non_normalized(p,pHit,pMiss,world,Z[0])
#j = normalize_p(new_p,sumarized)
#print(j)
print("-"*15)
#p1 = [1,0,0,0,0]
#print(move1(p1,2))

modify(p,n)
print(p)
print("\n")
for i in range(len(motions)):
    non_q, sumarized, new_p = non_normalized(p,pHit,pMiss,world,Z[i])
    print(new_p)
    p1 = normalize_p(new_p,sumarized)
    print(p1)
    new_pp = move(p1,motions[i])
    print(new_pp)
    p= new_pp
    print("\n")

    