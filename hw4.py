#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt


def computeGridSukharev(n):
    k = math.sqrt(n)
    sample_points = []
    if k % 1 != 0:
        print("n has to be a square number!")
        # Returns empty list if n != k^2 for some whole number k
    elif n<=0:
        print("n has to be a positive square number!")
    else:
        k = int(k)
        for i in range(k):
            for j in range(k):
                point = (1/(2*k)+i/k, 1/(2*k)+j/k,)
                sample_points.append(point)
    return sample_points


def computeGridRandom(n):
    sample_points = []
    if n <= 0:
        print("You have to choose a positive number!")
    else:
        # if n not a whole number we round it up either way
        n = math.ceil(n)
        for _ in range(n):
            x = np.random.uniform(low = 0, high = 1, size = 1)
            y = np.random.uniform(low = 0, high = 1, size = 1)
            sample_points.append((x,y,))
    return sample_points


def computeGridHalton(n, b1, b2):
    # Halton in 2D
    # Initiate a list with n sample points in the origin
    sample_points = [] 
    if n <= 0:
        print("You have to choose a positive number")
    else:
        x_points = [0 for i in range(n)]
        y_points = [0 for i in range(n)]
        for i in range(1, n+1): # Goes from 1 to n
            # Remember lists are 0-indexed
            i_tmpx  = i
            i_tmpy  = i
            fx      = 1/b1
            fy      = 1/b2
            # compute Halton for x
            while i_tmpx > 0:

                q = math.floor(i_tmpx/b1)
                r = i_tmpx%b1

                x_points[i-1] = x_points[i-1] + fx*r

                i_tmpx = q
                fx = fx/b1

            # Compute Halton for y
            while i_tmpy > 0:

                q = math.floor(i_tmpy/b2)
                r = i_tmpy%b2

                y_points[i-1] = y_points[i-1]+fy*r

                i_tmpy = q
                fy = fy/b2
            sample_points.append((x_points[i-1], y_points[i-1],))
        return sample_points

def test_grid_drawing(n):
    sukha_list      = computeGridSukharev(n)
    random_list     = computeGridRandom(n)
    halton_list     = computeGridHalton(n, 2, 3)

    x_sukha         = []
    y_sukha         = []
    x_rand          = []
    y_rand          = []
    x_halt          = []
    y_halt          = []

    for i in range(n):
        x_sukha.append(sukha_list[i][0])
        y_sukha.append(sukha_list[i][1])
        x_rand.append(random_list[i][0])
        y_rand.append(random_list[i][1])
        x_halt.append(halton_list[i][0])
        y_halt.append(halton_list[i][1])

   

    fig             = plt.figure(0)
    sub1            = fig.add_subplot(1,1,1)
    sub1.set_title('Sukharev uniform grid')
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.grid()
    plt.plot(x_sukha,y_sukha, 'ko')

    fig2            = plt.figure(1)
    sub2            = fig2.add_subplot(1,1,1)
    sub2.set_title('Uniform random grid')
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.grid()
    plt.plot(x_rand,y_rand, 'ko')

    fig3            = plt.figure(2)
    sub3            = fig3.add_subplot(1,1,1)
    sub3.set_title('Halton sequence grid')
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.grid()
    plt.plot(x_halt,y_halt, 'ko')
    plt.show()

test_grid_drawing(4)

    
