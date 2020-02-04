#!/usr/bin/env python3

from matplotlib import path
from hw1 import *

def computeDistancePointToPolygon(P, q):
    # P is array of tuples where the tuples are the coordinates of the vertices
    # The coordinates of the polygon is in order going counter-clockwise
    # q is tuple containing the coordinates of q
    # This function returns the shortest distance to a polygon
    polygon = path.Path(P)
    shortestDist = 100000000000000
    if polygon.contains_points([q])[0]:
        print("q is within the polygon P")
        return 0 #distance if q is within the polygon
    else:
        for i in range(len(P)): # Iterate through the vertices to find the line segment = edges
            if i == len(P)-1: # If last vertix
                dist = computeDistancePointToSegment(q, P[i], P[0])[0]
            else:
                dist = computeDistancePointToSegment(q, P[i], P[i+1])[0]
            if dist < shortestDist:
                # Find the shortest distance to the segment
                shortestDist = dist
        return floor(shortestDist*100)/100

def computeTangentVectorToPolygon(P, q):
    # if q is closest to a segment at the Polygon then it should return the unit length tangent vector parallell to
    # the segment
    # if q is closest to a vertix it should return the tangent vector to the circle with center at the vertix
    # and radius from the vertix to q
    polygon = path.Path(P)
    if polygon.contains_points([q])[0]:
        print("q is within the polygon P")
        return -1, -1  # No tangent vector if the point is inside the polygon
    else:
        typeOfShortestPoint = -1
        shortestDist = 100000000000
        shortestpoint1 = (0,0)
        shortestpoint2 = (0,0)
        for i in range(len(P)): # Iterate through the vertices to find the line segment = edges
            if i == len(P)-1: # If last vertix
                point1 = P[i]
                point2 = P[0]
                dist, shortestToPointType = computeDistancePointToSegment(q, point1, point2)
            else:
                point1 = P[i]
                point2 = P[i+1]
                dist, shortestToPointType = computeDistancePointToSegment(q, point1, point2)
            if dist < shortestDist:
                # Find the shortest distance to the segment
                if shortestToPointType != -1: # Update if we do not have vertices on top of each other
                    typeOfShortestPoint = shortestToPointType
                    shortestDist = dist
                    shortestpoint1 = point1
                    shortestpoint2 = point2

        # Calculating the unit vector:
        if typeOfShortestPoint == 0:
            # If shortest point is on segment
            scaling_factor = computeDistanceBetweenTwoPoints(shortestpoint1, shortestpoint2)

            # Tangent vector is counter-clockwise meaning it starts at p1 and goes at direction p2
            u              = ((shortestpoint2[0]-shortestpoint1[0])/scaling_factor,
                              (shortestpoint2[1]-shortestpoint1[1])/scaling_factor,)
        elif typeOfShortestPoint == 1:
            # if shortest point is on vertix p1
            vec = (q[0]-shortestpoint1[0], q[1]-shortestpoint1[1],)
            scaling_factor = (vec[0]**2+vec[1]**2)**(1/2)
            # Switch place and change the sign on one of the element to get the orthogonal vector
            # that is tangential on the circle
            u            = (-vec[1]/scaling_factor, vec[0]/scaling_factor)

        elif typeOfShortestPoint == 2:
            # if shortest point is on vertix p2
            vec = (q[0]-shortestpoint2[0], q[1]-shortestpoint2[1],)
            scaling_factor = (vec[0]**2+vec[1]**2)**(1/2)
            # Switch place and change the sign on one of the element to get the orthogonal vector
            # that is tangential on the circle
            u            = (-vec[1]/scaling_factor, vec[0]/scaling_factor)
        else:
            # Polygon is degenerate
            return -1, -2
        return u, typeOfShortestPoint

