#!/usr/bin/env python3

import queue as q
import math
# Input: a graph described by its adjacency table AdjTable and a start node
# Output: a list of pointers parents describing the BFS tree rooted at
# start
# AdjTable is a list of tuples where the i-th element in the list is a tuple
# containing the neighbor nodes of node i (i starts from 0), start is an integer between 0
# and leng(AdjTable)-1
def computeBFStree(AdjTable,start):
    # Create the tree by doing a BF search
    
    # list_of_parents[i]=j where i is the (i+1)-th node and j is the parent of node (i+1)
    list_of_parents = [] 

    #initialize empty queue:
    node_queue = q.Queue()
    # Insert start node
    node_queue.put(start) # Use 0-index element 0 in AdjTable is node 1

    for _ in range(len(AdjTable)):
        list_of_parents.append(None)
    list_of_parents[start] = start # SELF
    while node_queue.qsize() != 0:
        v = node_queue.get()
        for node in AdjTable[v]: # Get all the connecting nodes to v
            if list_of_parents[node] == None:
                list_of_parents[node] = v
                node_queue.put(node)
    
    return list_of_parents
    
    
    
    
    
    
# Input: a graph described by its adjacency table AdjTable, a start node
# and a goal node
# Output: a vector containing the nodes on a shortest path from the start
# to the goal
# AdjTable is a list of tuples where the i-th element in the list is a tuple
# containing the neighbor nodes of node i (i starts from 0), start is an integer between 0
# and leng(AdjTable)-1, goal is an integer between 0 and len(AdjTable)-1
def computeBFSpath(AdjTable,start,goal):
    # Create the path in reverse order
    list_of_parents = computeBFStree(AdjTable, start)
    
    list_P = [goal]
    u = goal


    # parent of u is not self
    while list_of_parents[u] != u:
        #print(u)
        u = list_of_parents[u]
        list_P.append(u)
    
    return list_P[::-1] # Return the path in correct order


adjacencyTable = [[1, 2, 3, 4],
                [0, 2],
                [0,1,5],
                [0, 4, 6],
                [0, 3, 5, 6],
                [2, 4, 9],
                [3, 4, 7],
                [6, 8, 9],
                [7, 9],
                [5, 7, 8]]

startNode = 0
goalNode = 7

# Input: two angles alpha and beta between [-pi,pi)
# Output: the distance theta between alpha and beta

def coputeDistanceOnCircle(alpha,beta):
    return (1/2)*((beta-alpha)%(2*math.pi)+(alpha-beta)%(2*math.pi)-abs((beta-alpha)%(2*math.pi)-(alpha-beta)%(2*math.pi)))
        
    
        
# Input: two points alpha and beta on 2-torus
# Output: the distance theta between alpha and beta
# alpha=(alpha1,alpha2) and beta=(beta1,beta2) are tuples

def computeDistanceOnTorus(alpha,beta):
    return math.sqrt(coputeDistanceOnCircle(alpha[0],beta[0])**2+coputeDistanceOnCircle(alpha[1],beta[1])**2)

