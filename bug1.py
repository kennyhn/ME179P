#!/usr/bin/env python3

import hw1
import hw2

from matplotlib import path
import matplotlib.pyplot as plt
import time
import math

def closestPolygon(current_position, polygonList):
    # Current position as a tuple (x,y) and list of all polygons
    minDistance = 100000000
    minPolygon  = None
    for polygon in polygonList:
        dist = hw2.computeDistancePointToPolygon(polygon, current_position)
        if dist < minDistance:
            minPolygon = polygon
            minDistance = dist

    # This only looks for the polygon closest to the robot in all directions
    # should be fixed so that it also checks the closest polygon that
    # intercept the line between current_position and goal
    return minPolygon, minDistance

def bugBase(startpoint, endpoint, path, obstacleslist, step_size):
    # INPUT: startpoint is a tuple (x_start,y_start) in W_free
    # INPUT: endpoint is a tuple (x_end, y_end) in W_free
    # INPUT: obstaclelist contains lists of polygons,
    #   Where each list in obstaclelist contains the vertices of the polygons
    # INPUT: step_size the maximum line-segment length between each points
    #   That is going to become the path of the robot
    # OUTPUT: A sequence/list of points from start to the first collision with a polygon
    #   that is if there is no polygons on the line between start-goal the list will be points
    #   going from start to goal
    # OUTPUT: Also returns the message "FAIL" or "SUCCESS" strings to say if you reached goal or not
    ## Q? Is it assumed that the obstacles are all in the way of the robot?
    startGoalDist       = hw1.computeDistanceBetweenTwoPoints(startpoint, endpoint)
    directionalVector   = (step_size*(endpoint[0]-startpoint[0])/startGoalDist,
                            step_size*(endpoint[1]-startpoint[1])/startGoalDist,)
    
    current_position    = startpoint

    while hw1.computeDistanceBetweenTwoPoints(current_position, endpoint) > step_size:
        polygonClosest, distanceToPolygon = closestPolygon(current_position, obstacleslist)
        if distanceToPolygon < step_size:
            return "Failure, There is an obstacle between the start and goal", path, polygonClosest
        current_position = (path[-1][0]+directionalVector[0], path[-1][1]+directionalVector[1],)
        path.append(current_position)
    path.append(endpoint)
    return "Success", path, None


def computeBug1(startpoint, goalpoint, obstacleslist, step_size):
    # INPUT: startpoint is a tuple (x_start,y_start) in W_free
    # INPUT: endpoint is a tuple (x_end, y_end) in W_free
    # INPUT: obstaclelist contains lists of polygons,
    #   Where each list in obstaclelist contains the vertices of the polygons
     # INPUT: step_size the maximum line-segment length between each points
    #   That is going to become the path of the robot
    start_time = time.time()
    path = [startpoint]
    message, path, obstacle = bugBase(startpoint, goalpoint, path, obstacleslist, step_size)

    removed_obstacles_list = []

    while "Failure," in message:
        path = circumnavigateObstacle(path[-1], goalpoint, path, obstacle, step_size)
        # remove obstacle from obstacleslist
        obstacleslist = [x for x in obstacleslist if x != obstacle]
        removed_obstacles_list.append(obstacle)
        # Continue from p_leave
        message, path, obstacle = bugBase(path[-1], goalpoint, path, obstacleslist, step_size)
    # Return fail if doing an illegal action e.g. crossing an obstacle
    
    for obstacle in removed_obstacles_list:
        for point in path:
            if hw2.computeTangentVectorToPolygon(obstacle, point)[0] == -1: # Inside polygon
                # If a point in path was in polygon it is wrong and returns fail
                return "Failure, no path was found from start to goal", []
            
    total_time = time.time() - start_time
    print("The code took {}s to execute".format(total_time))
    return "Success", path

def computeNextMove(current_point, obstacle, step_size):
    tan_vec, type_of_point = hw2.computeTangentVectorToPolygon(obstacle, current_point)
    next_point = (current_point[0]+tan_vec[0]*step_size, current_point[1]+tan_vec[1]*step_size)
    if type_of_point == 1 or type_of_point == 2: # If point on vertix decrease step_size
        next_point = (current_point[0]+tan_vec[0]*step_size*math.pow(10,-1), current_point[1]+tan_vec[1]*step_size*math.pow(10,-1),)
    return next_point

def circumnavigateObstacle(p_hit, goal_point, path, obstacle, step_size):
    # logic to go around obstacle and finding p_leave

    # Remember where we started
    leftStart = False

    start_point             = p_hit
    start_line_point        = computeNextMove(p_hit, obstacle, step_size)

    current_point           = start_point
    # Optimal point of leave
    p_leave          = p_hit
    shortest_distance_to_goal = hw1.computeDistanceBetweenTwoPoints(p_hit, goal_point)


    while True:
        # Once the robot reenters the line segment between start_point and start_line_point
        # The robot has completed one round around the obstacle
        if not leftStart and hw1.computeDistancePointToSegment(current_point, start_point, start_line_point)[0]>step_size:
            # Once it leaves the line segment at start, set leftStart to True
            leftStart = True
        
        current_point = computeNextMove(current_point, obstacle, step_size)
        path.append(current_point)
        distance_to_goal = hw1.computeDistanceBetweenTwoPoints(current_point, goal_point)
        if distance_to_goal < shortest_distance_to_goal:
            # Find point of leave
            p_leave = current_point
            shortest_distance_to_goal = distance_to_goal
        if (hw1.computeDistancePointToSegment(path[-1], start_point, start_line_point)[0] < 2*step_size) and leftStart:
            # Get out of this while loop
            break
    # Now the robot have to move to the point of leave
    while True:
        p_leave_line_point = computeNextMove(p_leave, obstacle, step_size)
        if hw1.computeDistancePointToSegment(path[-1], p_leave, p_leave_line_point)[0] < step_size:
            # The point of leave is necessarily at the exact point it has been at before
            break
        # move until at point of leave
        current_point = computeNextMove(current_point, obstacle, step_size)
        path.append(current_point)
    return path

def plot_robot_path(path, obstacles):
    fig1, _ = plt.subplots(1)
    for obstacle in obstacles:
        obs = plt.Polygon(obstacle, fc = 'g')
        plt.gca().add_patch(obs)
    
    x = []
    y = []
    for points in path:
        x.append(points[0])
        y.append(points[1])
    plt.plot(x[1:-2], y[1:-2], 'bo')
    plt.plot(x[0], y[0],'bx')
    plt.plot(x[-1], y[-1], 'bx')
    plt.show()

def calculate_path_length(path):
    total_distance = 0
    for i in range(len(path)-1):
        total_distance += hw1.computeDistanceBetweenTwoPoints(path[i], path[i+1])
    return total_distance


def plot_dist_vs_time(path):
    fig2, _ = plt.subplots(1)
    total_distance = calculate_path_length(path)
    y = []
    x = []
    for i in range(len(path)-1):
        y.append(hw1.computeDistanceBetweenTwoPoints(path[i], path[-1]))
        x.append(0.01*i)
    plt.title('Distance to goal as a function of time')
    plt.xlim(0,5)
    plt.ylim(0,6)
    plt.grid()
    plt.xlabel('t [s]')
    plt.ylabel('d [m]')
    plt.plot(x,y, 'g-')
    plt.show()

        

startPoint  = (0,0)
goalPoint   = (5,3)
step_size   = 0.1
obstaclesList = [((1,2,), (1,0,), (3,0,)), # Obstacle 1
                ((2,3,), (4,1,), (5,2,)),] #Obstacle 2

msg, path = computeBug1(startPoint, goalPoint, obstaclesList, step_size)
plot_robot_path(path, obstaclesList)
plot_dist_vs_time(path)
print("The total path length is {:.2f} units".format(calculate_path_length(path)))