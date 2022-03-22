# ENPM661 Project3 Phase1 A* Algorithm Implementation
import numpy as np
import cv2 as cv
import math
import copy

# function for calculate linear equation
def linear_eq(i,j,x1,y1,x2,y2):
    f = ((y2-y1)/(x2-x1))*(i-x1) + y1 - j
    return f

# function for backtrack optimal route
def backtrack(closed_list):
    route = [goal_node]
    while True:
        parent = copy.deepcopy(closed_list[route[0]][2])
        if parent == None:
            break
        else:
            route.insert(0,parent)
    return route

# function for getting priority queue
def get_priority(open_list):
    min = math.inf
    min_node = None
    for node in open_list:
        if open_list[node][0] + open_list[node][1] < min: # conpare total cost = C2C + C2G
            min = copy.deepcopy(open_list[node][0]) + copy.deepcopy(open_list[node][1]) 
            min_node = node
    return min_node    

# function for findig list of points between 2 nodes
def PtsInLine(node1,node2):
    list = []
    if node2[0] != node1[0]:
        for i in range(abs(node2[0]-node1[0])+1):
            if node2[0] > node1[0]:
                list.append([int(node1[0]+i), int(((node2[1]-node1[1])/(node2[0]-node1[0]))*i + node1[1])])
            else:
                list.append([int(node2[0]+i), int(((node2[1]-node1[1])/(node2[0]-node1[0]))*i + node2[1])])
    if node2[1] != node1[1]:
        for j in range(abs(node2[1]-node1[1])+1):
            if node2[1] > node1[1]:
                list.append([int(((node2[0]-node1[0])/(node2[1]-node1[1]))*j + node1[0]), int(node1[1]+j)])
            else:
                list.append([int(((node2[0]-node1[0])/(node2[1]-node1[1]))*j + node2[0]), int(node2[1]+j)])
    result = [] 
    for nodes in list: 
        if nodes not in result: 
            result.append(nodes) 
    return result

# function for actions
def ActionMove(current_node,angle_delta):
    i_current = copy.deepcopy(current_node[0])
    j_curent = copy.deepcopy(current_node[1])
    angle_next = current_node[2] - angle_delta
    # limit angle within +-360 deg
    while angle_next >= 360:
        angle_next = angle_next - 360
    while angle_next <= -360:
        angle_next = angle_next + 360
    # calculate position of next node
    i_next = int(i_current + L*math.cos(math.radians(angle_next)))
    j_next = int(j_curent + L*math.sin(math.radians(angle_next)))
    # get list of all points betweens current node and next node
    node_list = PtsInLine(current_node,(i_next,j_next))
    # check that all points in the list is within c-space
    for node in node_list:
        if (i_next>=map.shape[1] or i_next<0 or j_next>=map.shape[0] or j_next<0):
            status = False
            next_node = (i_current, j_curent, current_node[2])
            return [status,next_node]  
        if all(map[node[1],node[0]]==obstracle_cspace) or all(map[node[1],node[0]]==obstracle):
            status = False
            next_node = (i_current, j_curent, current_node[2]) 
            return [status,next_node]    
    status = True
    next_node = (i_next, j_next, angle_next)
    return [status,next_node]

# function for calculating heuristic function
def calC2G(node):
    c2g = math.sqrt((node[0] - x_goal)**2 + (node[1] - y_goal)**2)
    return c2g

# define colors of obstracles
obstracle = (255,0,0)
obstracle_cspace = (0,255,0)
visited = (255,255,255)

# initialize map
map = np.zeros((250,400,3))

# create obstracles
for i in range(map.shape[1]): # x-axis
    for j in range(map.shape[0]): # y-axis
        # circle obstracle
        if (i-300)**2 + (j-185)**2 - (40)**2 <= 0 :
            map[j,i] = obstracle
        # V obstracle
        V1 = linear_eq(i,j,*(36,185),*(105,100)) <= 0 and linear_eq(i,j,*(105,100),*(80,180)) >= 0 and linear_eq(i,j,*(80,180),*(36,185)) >= 0
        V2 = linear_eq(i,j,*(36,185),*(80,180),) <= 0 and linear_eq(i,j,*(80,180),*(115,210)) <= 0 and linear_eq(i,j,*(115,210),*(36,185)) >= 0
        if V1 or V2:
            map[j,i] = obstracle
        # hexagon obstracle
        if (linear_eq(i,j,*(200,60),*(235,80)) <= 0 and i<235 and linear_eq(i,j,*(235,120),*(200,140)) >= 0
            and linear_eq(i,j,*(200,140),*(165,120)) >=0 and i>165 and linear_eq(i,j,*(165,80),*(200,60)) <=0):
            map[j,i] = obstracle 

# display map with original obstracles            
map = cv.flip(map,0)
cv.imshow('map',map)
cv.waitKey(100) 
map = cv.flip(map,0)

# get robot radius and clearance from user input
while True:
    radius = input("Input robot radius : ")
    clearance = input("Input clearance between robot and obstracles : ")
    radius = int(float(radius))
    clearance = int(float(clearance))
    if radius < 0:
        print('radius is negative, please try again')
    elif clearance < 0:
        print('clearance is negative, please try again')
    else:    
        break    
    
# inflate obstracles with robot radius and clearance
map_temp = np.zeros((250,400,3))
for i in range(map.shape[1]): # x-axis
    for j in range(map.shape[0]): # y-axis
        if all(map[j,i] == obstracle):
            cv.circle(map_temp, (i,j), clearance+radius, obstracle_cspace, -1)
            
# redraw original obstracles
for i in range(map.shape[1]): # x-axis
    for j in range(map.shape[0]): # y-axis
        if all(map[j,i] == obstracle):
            map_temp[j,i] = obstracle
map = copy.deepcopy(map_temp)

# display inflated map           
map = cv.flip(map,0)
cv.imshow('map',map)
cv.waitKey(100) 
map = cv.flip(map,0)
            
# get start and goal node from user input
while True:
    x_start, y_start = input("Input start node (x_start, y_start) seperated by space : ").split()
    ang_start = input("Input start angle, as a multiple of 30 deg : ")
    x_goal, y_goal = input("Input goal node (x_goal, y_goal) seperated by space : ").split()
    ang_goal = input("Input goal angle, as a multiple of 30 deg : ")
    L = input("Input step size between 1 and 10 : ")
    x_start, y_start = int(x_start), int(y_start)
    ang_start = int(ang_start)
    x_goal, y_goal = int(x_goal), int(y_goal)
    ang_goal = int(ang_goal)
    L = int(float(L))
    # limit start and goal angle within +-360 deg
    while ang_start >= 360:
        ang_start = ang_start - 360
    while ang_start <= -360:
        ang_start = ang_start + 360
    while ang_goal >= 360:
        ang_goal = ang_goal - 360
    while ang_goal <= -360:
        ang_goal = ang_goal + 360
    if (x_start>=map.shape[1] or x_start<0 or y_start>=map.shape[0] or y_start<0):
        print('Start node is out of map, please try again')
    elif (x_goal>=map.shape[1] or x_goal<0 or y_goal>=map.shape[0] or y_goal<0):
        print('Goal node is out of map, please try again')
    elif all(map[y_start,x_start]==obstracle_cspace) or all(map[y_start,x_start]==obstracle):
        print('Start node is in obstracle, please try again')
    elif all(map[y_goal,x_goal]==obstracle_cspace) or all(map[y_goal,x_goal]==obstracle):
        print('Goal node is in obstracle, please try again')
    elif ang_start%30 != 0:
        print('start angle is not multiple of 30 deg, please try again')
    elif ang_goal%30 != 0:
        print('goal angle is not multiple of 30 deg, please try again')
    elif L < 1 or L > 10:
        print('step size is out of range, please try again')
    elif radius < 0:
        print('radius is negative, please try again')
    elif clearance < 0:
        print('clearance is negative, please try again')
    else:    
        break      
            
# initialize lists and nodes for A*
weight = 1 # 1 for A*
start_node = (x_start,y_start,ang_start)
goal_node = (x_goal,y_goal, ang_goal)
open_list = {} # dictionary format (x,y):[C2C,C2G,parent] 
open_list[start_node] = [0, calC2G(start_node), None]
closed_list = {} # dictionary format (x,y):[C2C,C2G,parent] 

# A* algorithm
while open_list != {}:
    # pop node with minimum total cost from open list, set as current node, and add to closed list
    current_node = get_priority(open_list)
    current_c2c = copy.deepcopy(open_list[current_node][0])
    closed_list[current_node] = open_list.pop(current_node)
    # check if current node is goal node (position toleatance = 1.5, angle tolerance = 30)
    if (math.sqrt((current_node[0] - goal_node[0])**2 + (current_node[1] - goal_node[1])**2) <= 1.5) and (abs(goal_node[2]-current_node[2]) <= 30):
        goal_node = current_node
        # track back parents and plot optimal route  
        route = backtrack(closed_list)
        print('Success')
        for i in range(len(route)-1):
            cv.line(map, (route[i][0],route[i][1]), (route[i+1][0],route[i+1][1]), (0,0,255), 1)
            map = cv.flip(map,0)             
            cv.imshow('map',map)
            cv.waitKey(10) 
            map = cv.flip(map,0)
        # display final map
        map = cv.flip(map,0)
        cv.imshow('map',map)
        cv.waitKey(0) 
        break
    # visit surrounding nodes and update their total cost, add them to open list if not already there
    else:
        # generate next nodes from all 5 actions
        result_list = []
        for angle_next in [-60, -30, 0, 30, 60]:
            next_empty, next_node = ActionMove(current_node,angle_next)
            result_list.append([next_empty, next_node])
        # check cost of next nodes
        for result in result_list:
            if (result[0] == True) and (result[1] not in closed_list):
                # draw line to possibe actions
                cv.line(map, (current_node[0],current_node[1]), (result[1][0],result[1][1]), (255,255,255), 1)
                if result[1] not in open_list:
                    # add new node to open list if not exist 
                    open_list[result[1]] = [current_c2c+L, weight*calC2G(result[1]), current_node]
                else:
                    # compare total cost if node already exist  
                    if open_list[result[1]][0] + open_list[result[1]][1] > current_c2c+L + weight*calC2G(result[1]):
                        open_list[result[1]] = [current_c2c+L, weight*calC2G(result[1]), current_node]
        
    # display current map
    cv.circle(map, (start_node[0],start_node[1]), 5, (0,0,255), -1)
    cv.circle(map, (goal_node[0],goal_node[1]), 5, (0,0,255), -1)
    map = cv.flip(map,0)               
    cv.imshow('map',map)
    cv.waitKey(1) 
    map = cv.flip(map,0)
else:
    # print failure if solution not found
    print('Failure') 

