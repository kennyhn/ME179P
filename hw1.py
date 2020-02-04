#!/usr/bin/env python3

from math import sqrt
from math import floor
import numpy as np

# As of now the code only works for points in R2

def checkPointsInR2(p):
    return len(p) == 2

def checkDistinctPoints(p1, p2, tol):
    if checkPointsInR2(p1) and checkPointsInR2(p2): # See if it is in R2
        # Return true if distinct points and false if not
        return not (computeDistanceBetweenTwoPoints(p1,p2) < tol)
    else:
        print("The points p1 and p2 are not in R2")
        return False

def computeDistanceBetweenTwoPoints(p1,p2):
    return sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)

def computeLineThroughTwoPoints(p1, p2): #input two points p1 and p2 as tuple of length 2

    tol = pow(10, -8)
    a, b, c = (0, 0, 0)

    if checkDistinctPoints(p1, p2, tol):
        x1, y1 = p1
        x2, y2 = p2

        if abs(x2-x1) > tol:
            # if the points are not ~vertical
            a1 = (y2-y1)/(x2-x1)
            b = 1
            # a*x+b*y+c=0 <=> y=a1*x+b1 when b=1, a = -a1, c = -b1
            a = -a1
            c = -a*x1-b*y1

            # find ratio m s.t. m^2*((a1)^2+ b1^2) = a^2+b^2 = 1
            m = sqrt(1/(a**2+b**2))

            # scale so that the parameters are normalized
            a = m*a
            b = m*b
            c = m*c
        else:
            # if x2-x1 < tol then the line is approx vertical
            a = 1
            b = 0
            c = -x1
        #print(("The line going through p1 = ({x1:.2f},{y1:.2f}) and p2 = ({x2:.2f}, {y2:.2f}) is given by"
        #      " the equation {a:.2f}x+{b:.2f}y+{c:.2f}=0").format(x1 = x1, y1 = y1, x2 = x2, y2 = y2,
        #                                                         a = a, b = b, c = c))
    else:
        print("The points p1 = ({},{}) and p2=({},{}) are not distinct".format(p1[0],p1[1],p2[0],p2[1]))
    return a, b, c

def computeDistancePointToLine(q, p1, p2):
    if checkDistinctPoints(p1, p2, pow(10, -8)):

        qx, qy = q
        x1, y1 = p1
        x2, y2 = p2
        vec1            = (x2-x1,y2-y1) # vector along the line
        vec2            = (-vec1[1], vec1[0]) # Use that orthogonal vector of vec1 is such that vec1 dot vec2 = 0


        # set up the linear equations equations such that we get p1+vec1*t = q+vec2*s
        A               = np.array([[vec1[0], -vec2[0]], [vec1[1], -vec2[1]]])
        b               = np.array([qx-x1, qy-y1])

        z               = np.linalg.solve(A,b)

        # Move along the line p1p2 to find the intersection point given by solution of lin.eq.
        intersect  = (x1+vec1[0]*z[0], y1+vec1[1]*z[0],)

        # compute distance from q to intersection point
        d          = computeDistanceBetweenTwoPoints(q,intersect)

        a,b,c = computeLineThroughTwoPoints(p1,p2)
        #print(("The distance from the point q = ({qx:.2f},{qy:.2f}) "
        #      "to the line {a:.2f}x+{b:.2f}y+{c:.2f}=0 is {dist:.2f}").format(qx = q[0], qy = q[1],
        #                                                                        a = a, b = b, c = c,
        #                                                                        dist = d))
        return d

    else:
        #print("The points p1 = ({},{}) and p2=({},{}) are not distinct".format(p1[0], p1[1], p2[0], p2[1]))
        return -1 # There is no line to find the distance to

def computeDistancePointToSegment(q, p1, p2):
    # Returns the distance d, and wether if the shortest distance is to point p1 or point p2 or segment

    tol = pow(10,-8)
    if checkDistinctPoints(p1, p2, tol):
        qx, qy          = q
        x1, y1          = p1
        x2, y2          = p2
        vec1            = (x2 - x1, y2 - y1)  # vector on the line
        vec2            = (-vec1[1], vec1[0])  # Use the orthogonal vector of vec1 st vec1 dot vec2 = 0

        # set up the linear equations equations such that we get p1+vec1*t = q+vec2*s
        A               = np.array([[vec1[0], -vec2[0]], [vec1[1], -vec2[1]]])
        b               = np.array([qx - x1, qy - y1])

        z               = np.linalg.solve(A, b)

        intersect       = (x1 + vec1[0] * z[0], y1 + vec1[1] * z[0],)

        distancep1top2          = computeDistanceBetweenTwoPoints(p1,p2)
        distancep1tointersect   = computeDistanceBetweenTwoPoints(p1,intersect)
        distancep2tointersect   = computeDistanceBetweenTwoPoints(intersect, p2)

        if distancep1tointersect+distancep2tointersect - distancep1top2 <= tol: # Intersection on line segment
            d    = computeDistanceBetweenTwoPoints(q, intersect)
            # if q is closest to line segment
            w    = 0
        else: # intersection outside of line segment
            d1 = computeDistanceBetweenTwoPoints(p1, q)
            d2 = computeDistanceBetweenTwoPoints(p2, q)
            if d1 < d2:
                d = d1
                # if q is closest to p1
                w = 1
            else:
                d = d2
                # if q is closest to p2
                w = 2

        #print(("The distance from the point q = ({qx:.2f},{qy:.2f}) "
        #       "to the line segment between p1 = ({x1:.2f}, {y1:.2f}) "
        #       "and p2 = ({x2:.2f}, {y2:.2f}) is {dist:.2f}").format(qx = q[0], qy = q[1],
        #                                                                x1 = x1, y1 = y1,
        #                                                                x2 = x2, y2 = y2, dist = d))
        return d, w
    else:
        print("The points p1 = ({},{}) and p2=({},{}) are not distinct".format(p1[0], p1[1], p2[0], p2[1]))
        # returns -1 for both if p1 and p2 are not distinct
        return -1, -1 # p1 and p2 not distinct





