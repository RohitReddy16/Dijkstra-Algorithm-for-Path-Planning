import cv2 as cv
import numpy as np
import math
import copy
import time 
from queue import PriorityQueue
#
clearance = 5
def linear_equation(i,j,x1,y1,x2,y2,clearance):
    if x2 - x1 == 0:
        # handle case where the denominator is zero
        if i == x1:
            return j - y1
        else:
            return float('inf')
    else:
        return ((y2 - y1) / (x2 - x1 + clearance)) * (i - x1 - clearance/2) + y1 - j
    
# def linear_equation(i, j, x1, y1, x2, y2, clearance):
#     try:
#         m = (y2 - y1) / (x2 - x1)
#         b = y1 - m * x1
#         f = m * i - j + b
#     except ZeroDivisionError:
#         f = float('inf')
#     if f > 0:
#         f -= clearance
#     else:
#         f += clearance
#     return f

obstracle = (255,0,0)
obstracle_cspace = (255,255,0)
visited = (255,255,255)

# initialize map
map = np.zeros((250,600,3))

#create the obstacles
for i in range(map.shape[1]):
    for j in range(map.shape[0]):
        #rectangle obstacle1
        if(i>=100 and i<=150 and j>=0 and j<=100):
            map[j,i] = obstracle
        #rectangle obstacle 2
        if(i>=100 and i<=150 and j>=150 and j<=250):
            map[j,i] = obstracle
        #hexagon obstacle
        if(linear_equation(i,j,*(300,50),*(364.95,87.5),clearance)<=0 and i<364.95 and linear_equation(i,j,*(364.95,162.5),*(300,200),clearance)>=0
           and linear_equation(i,j,*(300,200),*(235.05,162.5),clearance)>=0 and i>235.05 and linear_equation(i,j,*(235.05,87.5),*(300,50),clearance)<=0):
            map[j,i] = obstracle
        #triangle obstacle
        if (linear_equation(i,j,*(460,25),*(510,125),clearance) <= 0 and linear_equation(i,j,*(460,225),*(510,125),clearance) >= 0 and linear_equation(i,j,*(460,225),*(460,25),clearance) >= 0 and i>460):
            map[j,i] = obstracle
            
# display map with original obstracles            
map = cv.flip(map,0)
cv.imshow('map',map) 
map = cv.flip(map,0)
cv.waitKey(0)

