# -*- coding: utf-8 -*-
"""
Created on Sun Jul 09 23:38:49 2017
Search 
Graphs for pathfinder
A* Algorithm
@author: Eduardo
"""
#import math
#import collections
import heapq
grid2 = [[0,1],
        [0,0]]
#try with this and see the shortest path
grid4 = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]

grid = [[0, 1, 0, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 0]]

grid3 = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
start = (0,0)
goal = (len(grid)-1,len(grid[0])-1)
cost = 1
gvalue = 0
delta = [[-1,0], #up
         [0,-1], #left
         [1, 0], #down
         [0, 1]] #right

delta_name = ['^','<','v','>']

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
class graph:
    def __init__(self,height,width,landmarks):
        self.height = height
        self.width = width
        self.landmarks = []
        self.graph = self.create_graph()
    
    def in_bounds(self, results):
        xs=[]
        ys=[]
        new_results=[]
        
        for i in range(len(results)):
            xs.append(results[i][0])
            ys.append(results[i][1])
        for x in range(len(xs)):
            if 0 <= xs[x] < self.height and 0 <= ys[x] < self.width:
                new_results.append((xs[x],ys[x]))
        return new_results
    
    def passable(self, results):
        new_results = []
        save = []
        for i in landmarks:
            for j in results:
                verify = (i[0]==j[0])
                verify2 = i[1]==j[1]
                if verify and verify2:
                    #print "catcha:",i
                    save.append(j)
        if len(save)==0:
            return results            
        for i in range(len(save)):
            if i>0:
                new_results = self.passable(new_results)
            if i==0:                
                for coord in range(len(results)):
                    verify = (results[coord][0]==save[i][0])
                    verify2 = (results[coord][1]==save[i][1])
                    if verify and verify2:
                        pass
                    else:
                        new_results.append(results[coord])        
        #print "new: ", new_results
        return new_results
        

    def create_graph(self):
        graph = {}
        for x in range(self.height):
            for y in range(self.width):
                id = (x, y)
                if grid[x][y] !=1:               
                    results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
                    results = self.in_bounds(results)
                    #print id,results
                    results = self.passable(results)
                    graph[id] = results
        
        return graph

    def show_graph(self):       
        for key in self.graph:
            print("%s=%s \n"%(str(key),str(self.graph[key])))

        
def slicer(grid):
    for i in range(len(grid)):
        if len(grid[i]) != len(grid[i-1]):
            print("Your grid is invalid")
            raise ValueError   
    height = len(grid)
    width = len(grid[0])
    landmarks = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] != 0:
                landmarks.append((i,j))
    return height,width,landmarks

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal,cost):
    expand = [[-1]* len(grid[0]) for i in grid]
    frontier = PriorityQueue()
    frontier.put(start, heuristic(goal,start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    expand[start[0]][start[1]] = 0
    count = 1      
    while not frontier.empty():
        #print frontier.elements
        current = frontier.get()
        #print frontier.elements
        if current == goal:
            expand[current[0]][current[1]] = count
            break
        
        for key in graph.graph:
            verify = (key == current)
            if verify:
                key_1 = graph.graph[key]
                    
        for next in key_1:
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:    
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                if next != goal:
                    expand[next[0]][next[1]] = count #para ver la expansion general cambiar current por next
                    count += 1
                
    print("expand map:")            
    for i in range(len(expand)):
        print("%s\n"%(expand[i]))
    print(" ")
    print("camefrom=%s\n,cost_so_far=%s"%(came_from, cost_so_far))
    print(" ")
    for key in cost_so_far:
        verify = (key[0] == goal[0])
        verify2 = (key[1] == goal[1])
        if verify and verify2:
            winner = [cost_so_far[key],goal[0],goal[1]]
            return winner, came_from, cost_so_far
        
def reconstruct_path(came_from, start, goal,grid):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    #path.append(start) # optional
    path.reverse() # optional
    #draw the grid
    draw = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    for way in range(len(path)):
        for i in range(len(draw)):
            for j in range(len(draw[i])):
                verify = (i == path[way][0])
                verify2 = (j == path[way][1])
                if verify and verify2:
                    draw[i][j]= '*'                 
    #map
    for i in range(len(draw)):       
        print("%s\n"%(draw[i]))
    print(" ")
    return path
            
                 
height,width,landmarks = slicer(grid)
grapho = graph(height,width,landmarks)
grapho.show_graph()
print(" ")
#print grapho.graph
try:
    winner, came_from, cost_so_far = a_star_search(grapho, start, goal,cost)
    print("costo total del camino mas corto")
    print(winner)
    print("El camino que debe tomar:")
    print(reconstruct_path(came_from, start, goal,grid))
except Exception as e:
    print("fail",str(e))
    
input()