#!/usr/bin/env python3
import numpy as np
import math

# Input: two points p1 = (x1, y1) p2 = (x2, y2)

# Output: Distance between those two points
def computeDistanceBetweenTwoPoints(p1,p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)




# Input: two tuples containing the x and y direction of a vector
# Output: The dot product of those two vectors
def dot_product(vec1, vec2):
    return vec1[0]*vec2[0] + vec1[1]*vec2[1]




# Input: q=(x,y) is a point in 2D, P is list of tuples where 
# each element is the coordinate of a vertex of the convex polygon

# Output: TRUE if q is in P, FALSE otherwise
def isPointInConvexPolygon(q,P):
    # if a point is in the  convex polygon the dot product of  all the vectors from each vertix 
    # to the point q and the interior normal 
    # vector of the same vertix to the next (counter-clockwise)
    # vertix is positive.
    
    for i in range(len(P)): # Iterate through the vertices
        p1                = P[i]
        if i == (len(P)-1): # If P[i] last vertix in P
            p2 = P[0]
        else:
            p2 = P[i+1]

        inter_norm_vec    = (-(p2[1]-p1[1]), p2[0]-p1[0],)
        vertixtopoint_vec = (q[0]-p1[0], q[1]-p1[1],)

        if dot_product(inter_norm_vec, vertixtopoint_vec) < 0:
            # If one of the dot product are negative the point is outside the polygon
            return False

    # If no dot products were negative the point must be inside the convex polygon
    return True



    
# Input: p1 = (x1,y1), p2 = (x2,y2), p3 = (x3,y3), p4 = (x4,y4)
# are points in 2D

# Let the line segment defined by p1 and p2 be L1, 
# and the line segment defined by p3 and p4 be L2

# Output: TRUE and the intersection point if L1 intersects with L2;
# FALSE if L1 does not intersect with L2
def doTwoSegmentsIntersect(p1,p2,p3,p4):
    # The two lines p1p2 and p3p4 are defined by the parametrization
    # pa = p1 + s_a*(p2 - p1)
    # pb = p3 + s_b*(p4 - p3)
    # where if there exists an s_a and s_b between 0 and 1 then the line segments
    # intersects
    # let s_a = num/den. Then it can be shown that by setting pa = pb that for
    # p1 = (x1, y1), p2 = (x2, y2), p3 = (x3, y3) and p4 = (x4, y4)
    # num = (x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)
    # den = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    # where if num = den = 0 the line segments are coincident -> may intersect
    # num != 0 and den = 0 the two lines are parallell and distinct -> do not intersect
    # if num != 0 and den != 0, then the two lines are not parallel and may intersect
    num = (p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])
    den = (p4[1]-p3[1])*(p2[0]-p1[0]) - (p4[0]-p3[0])*(p2[1]-p1[1])
    if num == 0 and den == 0:
        # The lines are coincident
        # Need to check if they actually intersect with each other
        # if one of the line vertices are inside the other line they intersect, otherwise they do not
        p1p2 = computeDistanceBetweenTwoPoints(p1, p2)
        p3p4 = computeDistanceBetweenTwoPoints(p3, p4)

        p1p3 = computeDistanceBetweenTwoPoints(p1, p3)
        p2p3 = computeDistanceBetweenTwoPoints(p2, p3)
        p1p4 = computeDistanceBetweenTwoPoints(p1, p4)
        p2p4 = computeDistanceBetweenTwoPoints(p2, p4)
        p3p4 = computeDistanceBetweenTwoPoints(p3, p4)
        if p1p3 + p2p3 == p1p2:
            # p3 are inside line p1p2
            return True
        elif p1p4 + p2p4 == p1p2:
            # p4 are inside line p1p2
            return True
        elif p1p3 + p1p4 == p3p4:
            # p1 is inside line p3p4
            return True
        elif p2p3 + p2p4 == p3p4:
            # p2 is inside line p3p4
            return True
        else:
            # no line vertices are in the other segment
            return False

    if num != 0 and den == 0:
        # The lines are parallell and distinct
        return False
    else:
        # Have to check if they actually intersect
        # We solve s_a and s_b to see if they are both between 0 and 1
        A = np.array([[p2[0]-p1[0], -(p4[0]-p3[0])], [p2[1]-p1[1], -(p4[1]-p3[1])]])
        b = np.array([p3[0]-p1[0], p3[1]-p1[1]])
        s_a, s_b = np.linalg.solve(A, b)
        if (s_a <= 1 and s_a >= 0) and (s_b <= 1 and s_b >= 0):
            return True # they do intersect
        else:
            return False




# Input: P1 and P2 are two lists of tuples where each element of a
# list is the coordinate of a vertex of the convex polygon

# Output: TRUE if P1 intersects with P2; FALSE otherwise.
def doTwoConvexPolygonsIntersect(P1,P2):
    # Collision if any vertices of P1 is in P2
    for vertixP1 in P1:
        if isPointInConvexPolygon(vertixP1, P2):
            return True

    # Collision if any vertices of P2 is in P1
    for vertixP2 in P2:
        if isPointInConvexPolygon(vertixP2, P1):
            return True

    # Collision if any edge of P1 intersects any edge of P2
    for vertixp1_nr in range(len(P1)):
        # find two following vertices in P1
        vertix1poly1 = P1[vertixp1_nr]
        if vertixp1_nr == len(P1)-1: # If last vertix
            vertix2poly1 = P1[0]
        else:
            vertix2poly1 = P1[vertixp1_nr+1]

        for vertixp2_nr in range(len(P2)):
            vertix1poly2 = P2[vertixp2_nr]
            if vertixp2_nr == len(P2)-1: # If last vertix
                vertix2poly2 = P2[0]
            else:
                vertix2poly2 = P2[vertixp2_nr+1]
            
            if doTwoSegmentsIntersect(vertix1poly1, vertix2poly1, vertix1poly2, vertix2poly2):
                return True # If one of the edges in the polygons intersects the polygons intersect
    
    # If no True has returned then there is no collisions
    return False

