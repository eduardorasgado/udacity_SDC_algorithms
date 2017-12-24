delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>'] # Use these when creating your policy grid.

# ---------------------------------------------
#  Modify the function stochastic_value below
# ---------------------------------------------

def stochastic_value(grid,goal,cost_step,collision_cost,success_prob):
    failure_prob = (1.0 - success_prob)/2.0 # Probability(stepping left) = prob(stepping right) = failure_prob
    value = [[collision_cost for col in range(len(grid[0]))] for row in range(len(grid))]
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    change = True
    while change == True:
        change = False
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                
                if x==goal[0] and y == goal[1]:
                    if value[x][y]>0:
                        value[x][y]=0.0
                        policy[x][y]='*'
                        change=True
                        
                elif grid[x][y]==0:                   
                    for a in range(len(delta)):
                        v2 =cost_step
                        #exploramos las acciones
                        for i in range(-1,2):
                            a2 = (a+i)% len(delta)
                            x2 = x+ delta[a2][0]
                            y2 = y + delta[a2][1]
                            if i==0:
                                p2 = success_prob
                            else:
                                p2 = failure_prob
                            if x2>=0 and x2<len(grid) and y2>=0 and y2<len(grid[0]) and grid[x2][y2]==0:
                                v2 += p2 * value[x2][y2]
                            else:
                                v2+= p2*collision_cost
                        
                        if v2<value[x][y]:
                            value[x][y]=v2
                            policy[x][y]=delta_name[a]
                            change=True    
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            value[x][y] = round(value[x][y],2)
    
    return value, policy


grid0 = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 1, 0]]

grid = [[0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]]

goal = [4, 7]
cost_step = 1
collision_cost = 100
success_prob = 0.5

value,policy = stochastic_value(grid,goal,cost_step,collision_cost,success_prob)
for row in value:
    print(row)
for row in policy:
    print(row)
input()