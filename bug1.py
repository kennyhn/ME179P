#!/usr/bin/env python3

import hw1
import hw2


def closestPolygon(current_position, directionalUnitVector, polygonList):
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

def bugBase(startpoint, endpoint, obstaclelist, step_size):
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
    startgoalDist       = ((endpoint[0]-startpoint[0])**2+(endpoint[1]-startpoint[1])**2)**(1/2)
    directionalVector   = (step_size*(endpoint[0]-startpoint[0])/startGoalDist,
                            step_size*(endpoint[1]-startpoint[1])/startGoalDist,)
    current_position    = startpoint
    path                = [startpoint] # list of points (x,y)

    while hw1.computeDistanceBetweenTwoPoints(startpoint, endpoint) > step_size:
        polygonClosest, distanceToPolygon = closestPolygon(current_position, polygonList):
        if distanceToPolygon < step_size:
            # need an extra check to see if the polygon is actually in the path between start and goal
            return "Failure, There is an obstacle between the start and goal", path
        current_position = (current_position[0]+directionalVector[0], current_position[1]+directionalVector[1])
        path.append(current_position)
    path.append(endpoint)    

    return "Success", path

