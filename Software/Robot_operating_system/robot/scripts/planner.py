#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (C) 2013, Agustin Caverzasi, Fernando Saravia Rajal,
# Laboratorio de Arquitectura de Computadoras (LAC), 
# Grupo de Robotica y Sistemas Integrados (GRSI),
# Universidad Nacional de Córdoba (UNC), Cordoba, Argentina. 
# All rights reserved.
 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Contact information
#
# Agustin Caverzasi, UNC
# e-mail   :  agucaverzasi@gmail.com
# 
# Fernando Saravia Rajal, UNC
# e-mail   :  fersarr@gmail.com
# 
# Updates should always be available at https://github.com/agucaverzasi/Robot-movil-mapeo3D
#
# Changelog:
#           01/07/2013 file added to repository
#
# This script contains code that was extracted from the online CS 373: Programming a Robotic 
# Car course given by Sebastian Thrun at Udacity.com. 
#

import roslib; roslib.load_manifest('robot')
import rospy
from std_msgs.msg import String
from math import *
import random
import matplotlib.pyplot as mtpl

# search function returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. 

#------------------------
#	init and goal coordinates 
#	to compute path from map file
#

#-------------------------
#	Test1
#
#init=[0,0]		#(map2.dat)
#goal=[5,5]		#(map2.dat) 
#map_file="map2.dat"

#-------------------------
#	Test2 (habitacion)
#
#init=[50,50] 	#(map.dat)
#goal=[35,33] 	#(map.dat)   	
#map_file="map.dat"

#-------------------------
#	Test3 (LAC)
#
init=[50,50] 	#(map1.dat)
#goal=[57,79]	#box norte
#goal=[57,22] 	#puerta
goal=[77,37]	#
#goal=[37,23] 	#final pasillo
#goal=[61,23] 	#puerta saliendo
map_file="map1.dat"

#-------------------------
#	Test4
#
#init=[0,0]		#(map3.dat)
#goal=[4,5]		#(map3.dat) 
#map_file="map3.dat"

#-----------------------------------------
#	heuristic function for A* algorithm
#	estimate the distance from each node 
#	to goal. Notice that heuristic funtion
#	have to be the same size as map matrix.
#
heuristic = [[9, 8, 7, 6, 5, 4],
            [8, 7, 6, 5, 4, 3],
            [7, 6, 5, 4, 3, 2],
            [6, 5, 4, 3, 2, 1],
            [5, 4, 3, 2, 1, 0]]
#------------------------------------
#	robot movements
#
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right
         
motions = ['^', '<', 'v', '>']			#robot motions
move = ['up', 'left', 'down','right']	#robot commands

cost = 10								#measured in [cm]

  
#-----------------------------
#	path search function (A*)
#
def search():
	#-----------------------------
	#	print map(grid) size
	#
    print "Grid rows: %d" % len(grid)
    print "Grid columns: %d" % len(grid[0])
   
    #-----------------------------
    #	A* algorithm (compute shortest path)
    #
    closed = [[0 for row in range(len(grid))] for col in range(len(grid[0]))]
    action = [[-1 for row in range(len(grid))] for col in range(len(grid[0]))]
    grid_map = [[' ' for row in range(len(grid))] for col in range(len(grid[0]))]
    closed[init[0]][init[1]] = 1
    x = init[0]
    y = init[1]
    g = 0
    open = [[g, x, y]]
	
    found = False  # flag that is set when search is complet
    resign = False # flag set if we can't find expand
	
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            g = next[0]

            if ( (x == goal[0]) and (y == goal[1]) ):
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    
                    if ( (x2 >= 0) and (x2 < len(grid[0])) and (y2 >=0) and (y2 < len(grid)) ):
                    	#print x2,y2
                    	if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                        	g2 = g + cost #+ heuristic[x2][y2]			# without heuristic function is a simple BFS
                        	open.append([g2, x2, y2])
                        	closed[x2][y2] = 1
                        	action[x2][y2] = i
    
    #------------------------------------
    #	get the path in coordinates, robot movements and road map
    #
	path_coord = []
	path_move = []
    x = goal[0]
    y = goal[1]
    grid_map[x][y] = '*'
    path_coord.append([x, y])
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        x = x2
        y = y2
        grid_map[x2][y2] = motions[action[x2][y2]]
        path_move.append(move[action[x2][y2]])
        path_coord.append([x2, y2])
    
    path_move=path_move[::-1]		#reverse list
    path_coord=path_coord[::-1]		#reverse list
        
    #------------------------------------
    #	first movement correction	    
    #
    if (grid_map[init[0]][init[1]-1]!=' '):
    	grid_map[init[0]][init[1]]='<'
    	path_move[0]='left'
    elif (grid_map[init[0]][init[1]+1]!=' '):
    	grid_map[init[0]][init[1]]='>'
    	path_move[0]='right'
    elif (grid_map[init[0]-1][init[1]]!=' '):
    	grid_map[init[0]][init[1]]='^'
    	path_move[0]='up'
    elif (grid_map[init[0]+1][init[1]]!=' '):
    	grid_map[init[0]][init[1]]='v'
    	path_move[0]='down'    
    
    #------------------------------------
    #	print functions for testing
	#
	
    #print action matrix
    #for i in range(len(action)):
    #    print action[i]
    
	#print grid with path
    #for i in range(len(grid_map)):
    #    print grid_map[i]
   	
    #print path coordinates
    print 'path_coord = ', path_coord

    #print path commands
    print 'path_move = ', path_move
    
    #------------------------------------
	#	get the smoothing coordinates
	#
    spath_coord=[]
    #spath_coord=smooth(0.1,0.2,0.000001,path_coord)
    
    #------------------------------------
    #	print 2D map with robot trajectory
    #
    map_2d(grid_map,path_coord,spath_coord)
    
    return path_move


# ------------------------------------------------
# 
# this is the smoothing function
#
def smooth(weight_data, weight_smooth, tolerance, path_coord):

    if path_coord == []:
        raise ValueError, "Run A* first before smoothing path"

    spath = [[0 for row in range(len(path_coord[0]))] for col in range(len(path_coord))]
    for i in range(len(path_coord)):
        for j in range(len(path_coord[0])):
            spath[i][j] = path_coord[i][j]

    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path_coord)-1):
            for j in range(len(path_coord[0])):
                aux = spath[i][j]
                
                spath[i][j] += weight_data * \
                    (path_coord[i][j] - spath[i][j])
                
                spath[i][j] += weight_smooth * (spath[i-1][j] + spath[i+1][j] - (2.0 * spath[i][j]))
                if i >= 2:
                    spath[i][j] += 0.5 * weight_smooth * (2.0 * spath[i-1][j] - spath[i-2][j] - spath[i][j])
                if i <= len(path_coord) - 3:
                    spath[i][j] += 0.5 * weight_smooth * \
                        (2.0 * spath[i+1][j] - spath[i+2][j] - spath[i][j])
            
        change += abs(aux - spath[i][j])
    return spath


#-----------------------------
#	call A*, tranform path and send commands
#
def get_path():
    #---------------------------
    #	ROS communication
    #
    pub = rospy.Publisher('commands', String)
    rospy.init_node('planner', anonymous=True)
    r = rospy.Rate(10) # 10hz
    
    #---------------------------
    #	call A* to get the robot path
    #
    path = search()
    
    #---------------------------
    #	transform path to real robot commands
    #
    real_path=list(path)
    i=0
    for x in range(len(path)-1):
    	if (   (path[x]=='up'    and path[x+1]=='up'   )   or \
    		   (path[x]=='down'  and path[x+1]=='down' )   or \
    		   (path[x]=='right' and path[x+1]=='right')   or \
    		   (path[x]=='left'  and path[x+1]=='left' )):
    		   	real_path[i]='up'
    			i+=1
    			#print i, i+1, 'up', "i= ",i
    			#print real_path
    			#print
    	elif ( (path[x]=='down'  and path[x+1]=='up'   )   or \
    		   (path[x]=='up'    and path[x+1]=='down' )):
    			real_path[i]='down'
    			i+=1
    			#print i, i+1, 'down', "i= ",i
    			#print real_path
    			#print
    	elif ( (path[x]=='down'  and path[x+1]=='left' )   or \
    		   (path[x]=='up'    and path[x+1]=='right')   or \
    		   (path[x]=='right' and path[x+1]=='down')   or \
    		   (path[x]=='left'  and path[x+1]=='up' )):
    			real_path[i]='right'
    			i+=1
    			real_path.insert(i,'up')
    			i+=1
    			#print i, i+1, 'right + up', "i= ",i
    			#print auxpath
    			#print
    	elif ( (path[x]=='down'  and path[x+1]=='right')   or \
    		   (path[x]=='up'    and path[x+1]=='left' )   or \
    		   (path[x]=='left'    and path[x+1]=='down' )   or \
    		   (path[x]=='right' and path[x+1]=='up'   )): 
    			real_path[i]='left'
    			i+=1
    			real_path.insert(i,'up')
    			i+=1
    			#print i, i+1, 'left + up', "i= ",i
    			#print auxpath
    			#print
    	else:
    		print "None command"
    	
    #-------------------------
    #	add init and end commands
    #
    real_path.insert(0,'init')
    real_path[len(real_path)-1]='stop'
    
    #-------------------------
    #	print final path for robot
    #
    print 'real_path(robot)=',real_path
    
    #-------------------------
    #	send realpath to robot node (publish movements)
    #    
    i=0
    while not rospy.is_shutdown() and i<len(real_path):
        str = real_path[i]
        i=i+1
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()


#-----------------------------
#	get the grid map from file
#
def read_file():
	#-------------------------
	#	read map from file
	#
	mapp = []
	with open(map_file) as f:
	    print "Reading map..."
	    arch=f.readlines()
	    for i in range(len(arch)):
	    	raw=[]
	    	for c in arch[i]:
	    		if (c!='\n' and c!='\t' and c!=' '):
	    			raw.append(int(c))
	    	mapp.append(raw)
	    return mapp	

#-----------------------------
#	search goal
#	    
def get_goal():
	for i in range(len(grid)):
		x=i
    	y=grid[i].index(7)			
	return x,y

# ------------------------------------------------
# 
# create a 2d map and prints the robot trayectory
#
def map_2d(carhistory=[], path_coord=[], spath_coord=[]):
	mtpl.figure()
	v=[0, 100, 0, 100]
	mtpl.axis(v)
	for i in range(len(grid)):
		for j in range(len(grid[0])):
			if grid[i][j]==1: mtpl.plot(i,j,'ro')
	for i in range(len(carhistory)):
		for j in range(len(carhistory[0])):
			if carhistory[i][j]=='>': mtpl.plot(i,j)
			elif carhistory[i][j]=='<': mtpl.plot(i,j)
			elif carhistory[i][j]=='v': mtpl.plot(i,j)
			elif carhistory[i][j]=='^': mtpl.plot(i,j)
	mtpl.plot([c[0] for c in path_coord], [c[1] for c in path_coord], label='Robot path')
	if not path_coord == []:
		mtpl.plot([c[0] for c in spath_coord], [c[1] for c in spath_coord], label='Smooth Robot path')
	mtpl.legend()
	mtpl.show()


#-----------------------------
#	main function
#
if __name__ == '__main__':
    try:
    	grid=read_file()
    	final_path=get_path()
    	#print get_goal()
    except rospy.ROSInterruptException: pass





