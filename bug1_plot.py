#!/usr/bin/env python3

import hw1
from bug1 import *

import matplotlib.pyplot as plt


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
    plt.title('Robot path in W_free', fontsize = 32)
    plt.grid()
    plt.plot(x[1:-2], y[1:-2], 'bo')
    plt.plot(x[0], y[0],'bx', markersize = 12)
    plt.plot(x[-1], y[-1], 'bx', markersize = 12)
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
    plt.title('Distance to goal as a function of time', fontsize = 32)
    plt.xlim(0,5)
    plt.ylim(0,6)
    plt.grid()
    plt.xlabel('t [s]', fontsize = 24)
    plt.ylabel('d [m]', fontsize = 24)
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