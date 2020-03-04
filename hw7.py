#!/usr/bin/env python3
import math
import numpy as np

def isThetaValid(theta):
    # For angle axis rotation
    # Check if theta is between 0 and pi
    
    if theta > math.pi:
        return False
    if theta < 0:
        return False
    return True

def isVectorUnitLength(n):
    # INPUT: numpy array of size nx1
    num_elements, _ = np.shape(n)
    length          = 0
    for i in range(num_elements):
        length += n[i, 0]**2
    length = math.sqrt(length)
    if abs(length-1) <= 10**(-8): # Vector is unit length
        return True
    else:
        return False

def calculateSkewMatrixFromVector(n):
    # INPUT: A 3x1 numpy array n (vector)
    # OUTPUT: A skew symmetric matrix as defined in the book

    skewN = np.array([[0, -n[2, 0], n[1, 0]],
                    [n[2, 0], 0, -n[0, 0]],
                    [-n[1, 0], n[0,0], 0]])
    return skewN 

def calculateVectorFromSkewMatrix(skewMat):
    # INPUT: A 3x3 skewsymmetric matrix as numpy array
    # OUTPUT: A 3x1 numpy array or -1 if skewmatrix is not valid

    if skewMat[0, 0] == 0 and skewMat[1, 1] == 0 and skewMat[2, 2] == 0:
        if skewMat[0, 1] == -skewMat[1, 0] and skewMat[0, 2] == -skewMat[2, 0] and skewMat[1, 2] == -skewMat[2, 1]:
            return np.array([[-skewMat[1, 2]], [skewMat[0, 2]], [-skewMat[0, 1]]])
        else:
            return -1 #Invalid skew symmetric matrix
    else:
        return -1 #Invalid skew symmetric matrix

def calculateTraceOfMatrix(matrix):
    rows, cols = np.shape(matrix)
    if rows != cols:
        # Trace is only defined for square matrices
        return -1
    trace = 0
    for i in range(rows):
        trace += matrix[i, i]
    return trace


def computeRMfromAA(theta, n):
    # INPUT: angle theta between 0 and pi
    # INPUT: vector n with 3 elements (numpy vector 3x1)
    # OUTPUT: 3x3 Rotation matrix as a numpy array
    vector_length = 0
    num_elements, _ = np.shape(n)
    if not isThetaValid(theta):
        return -1 #invalid theta angle

    if not isVectorUnitLength(n): # make vector unit length
        for i in range(num_elements):
            vector_length += n[i, 0]**2
        vector_length = math.sqrt(vector_length)
        n             = n/vector_length
    
    #### Calculate Rotation matrix ####
    identity_3  = np.identity(3)
    skew_n      = calculateSkewMatrixFromVector(n)
    skew_n_sq   = np.matmul(skew_n, skew_n)

    result      = np.add(identity_3, math.sin(theta)*skew_n)
    result      = np.add(result, (1-math.cos(theta))*skew_n_sq)
    
    # Clean rotation matrix for smaaaalll values
    rows, cols = np.shape(result)
    for i in range(rows):
        for j in range(cols):
            if abs(result[i, j]) <= 10**(-8):
                result[i, j] = 0
    return result


def computeAAfromRM(rotation_mat):
    # INPUT: 3x3 rotation_matrix
    # OUTPUT: angle theta and vector n as a tuple where n is a 3x1 numpy array
    # OUTPUT: if rotation matrix is not valid it returns (-1, -1)
    
    theta = -1
    n     = -1

    # Check if determinant of rotation matrix is +1
    if abs(np.linalg.det(rotation_mat)-1) > 10**(-8):
        # not a rotation matrix
        return (theta, n,)
    
    rows, cols = np.shape(rotation_mat)

    r_transr   = np.matmul(np.transpose(rotation_mat), rotation_mat)

    # Check if R^T*R == I
    identityMatrix = True
    for i in range(rows):
        for j in range(cols):  
            if i==j:
                if abs(r_transr[i, j] - 1) > 10**(-8):
                    # Diagonal elements are not 1
                    # means that rotation matrix is not identity
                    identityMatrix = False
            else:
                if abs(r_transr[i, j]) > 10**(-8):
                    # not a rotation matrix
                    # Off-diagonal elements are not 0
                    identityMatrix = False
    
    if not identityMatrix:
        # Not a valid rotation matrix
        return (theta, n,)
    # From here rotation_mat is element of SO(n) so it is a valid rotation matrix

    isRotationIdentity = True
    for i in range(rows):
        for j in range(cols):  
            if i==j:
                if abs(rotation_mat[i, j] - 1) > 10**(-8):
                    # Diagonal elements are not 1
                    isRotationIdentity = False
            else:
                if abs(rotation_mat[i, j]) > 10**(-8):
                    # Off-diagonal elements are not 0
                    isRotationIdentity = False
    # Case i)
    if isRotationIdentity:
        # There is an infinite number of solutions where theta = 0 and axis is arbitrary
        # In this case we set the axis as the 0-vector
        theta   = 0
        n       = np.array([[0], [0], [0]])
        return (theta, n,)

    # Case ii)
    traceRotMat = calculateTraceOfMatrix(rotation_mat)
    traceRotMat = round(traceRotMat*(10**8))/(10**8) # Round the number
    if  traceRotMat < 3 and traceRotMat > -1:
        theta = math.acos((traceRotMat-1)/2)
        n     = 1/(2*math.sin(theta))*calculateVectorFromSkewMatrix(rotation_mat-np.transpose(rotation_mat))
    # Problem description says to ignore case iii)

    return (theta, n,)


def main():
    # insert angle here
    theta = math.pi
    # insert vector here
    n     = np.array([[1], [1], [1]]) # Function normalizes the vectors no matter length

    # Test functions
    rot_mat = computeRMfromAA(theta, n)
    # Print the result
    print(rot_mat)
    theta_r, n_r = computeAAfromRM(rot_mat)
    print(f'theta: {theta_r}\nvector: {n_r}')
    return 0


if __name__ == "__main__":
    main()
