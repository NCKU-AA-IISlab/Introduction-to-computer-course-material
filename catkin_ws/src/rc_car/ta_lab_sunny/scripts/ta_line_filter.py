import numpy as np
from scipy.stats import multivariate_normal, entropy
from scipy.ndimage.filters import gaussian_filter
from math import floor, atan2, pi, cos, sin, sqrt
import time
from matplotlib import pyplot as plt


#Environment setup
# constant
WHITE = 0
YELLOW = 1
RED = 2

lanewidth = 0.4
linewidth_white = 0.04
linewidth_yellow = 0.02

#Generate Vote from Line Segment

# right edge of yellow lane
p1 = np.array([0.8, 0.24])
p2 = np.array([0.4, 0.24])
#print len(lines_normalized)
#p1 = np.array([lines_normalized[0][0],lines_normalized[0][1]])
#p2 = np.array([lines_normalized[0][2],lines_normalized[0][3]]) 
seg_color = YELLOW
print p1[0], p1[1], p2[0], p2[1]

# left edge of white lane
#p1 = np.array([0.4, 0.2])
#p2 = np.array([0.8, 0.2])
#seg_color = WHITE

plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'ro')
plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
plt.ylabel('y')
plt.axis([0, 1, 0, 1])
plt.show()
#compute d_i,phi_i , l_i
t_hat = (p2-p1)/np.linalg.norm(p2-p1)
n_hat = np.array([-t_hat[1],t_hat[0]])
d1 = np.inner(n_hat,p1)
d2 = np.inner(n_hat,p2)
l1 = np.inner(t_hat,p1)
l2 = np.inner(t_hat,p2)

print (d1, d2, l1, l2)

if (l1 < 0):
    l1 = -l1;
if (l2 < 0):
    l2 = -l2;
l_i = (l1+l2)/2
d_i = (d1+d2)/2
phi_i = np.arcsin(t_hat[1])
if seg_color == WHITE: # right lane is white
    if(p1[0] > p2[0]): # right edge of white lane
        d_i = d_i - linewidth_white
        print ('right edge of white lane')
    else: # left edge of white lane
        d_i = - d_i
        phi_i = -phi_i
        print ('left edge of white lane')
    d_i = d_i - lanewidth/2

elif seg_color == YELLOW: # left lane is yellow
    if (p2[0] > p1[0]): # left edge of yellow lane
        d_i = d_i - linewidth_yellow
        phi_i = -phi_i
        print ('left edge of yellow lane')
    else: # right edge of yellow lane
        d_i = -d_i
        print ('right edge of yellow lane')
    d_i =  lanewidth/2 - d_i

    
print (d_i, phi_i, l_i)
