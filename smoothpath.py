# -----------
# User Instructions
#
# Define a function smooth that takes a path as its input
# (with optional parameters for weight_data, weight_smooth,
# and tolerance) and returns a smooth path. The first and 
# last points should remain unchanged.
#
# Smoothing should be implemented by iteratively updating
# each entry in newpath until some desired level of accuracy
# is reached. The update should be done according to the
# gradient descent equations given in the instructor's note
# below (the equations given in the video are not quite 
# correct).
# -----------

from copy import deepcopy
import matplotlib.pyplot as plt
from matplotlib import style

style.use("fivethirtyeight")

# thank you to EnTerr for posting this on our discussion forum
def printpaths(path,newpath):
    for old,new in zip(path,newpath):
        print('['+ ', '.join('%.3f'%x for x in old) + '] -> ['+ ', '.join('%.3f'%x for x in new) + ']')

# Don't modify path inside your function.
path = [[0, 0],
        [0, 1],
        [0, 2],
        [1, 2],
        [2, 2],
        [3, 2],
        [4, 2],
        [4, 3],
        [4, 4]]

def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):
    #weight_Dta is alpha
    #weight_smooth is beta
    # Make a deep copy of path into newpath
    newpath = deepcopy(path)
    #######################
    change = tolerance
    while change >= tolerance:
        change= 0.0
        for i in range(1,len(path)-1):
            for j in range(len(path[0])):
                helper = newpath[i][j]
                newpath[i][j] += weight_data*(path[i][j] - newpath[i][j]) + weight_smooth*(newpath[i-1][j] +  newpath[i+1][j] - 2.0 *newpath[i][j])
                change+= abs(helper-newpath[i][j])
                
    return newpath # Leave this line for the grader!

printpaths(path,smooth(path))

path1 = smooth(path)
x1 = []
y1 = []

for i in range(len(path1)):
    x1.append(path1[i][0])  
    y1.append(path1[i][1])
    
plt.plot(x1,y1,'r--',label="path 1")
plt.legend(loc=4)
plt.show()
