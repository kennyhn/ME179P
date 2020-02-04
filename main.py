#!/usr/bin/env python3

import numpy as np
from hw2 import *

def main():
     p1 = ((1,2), (1,0), (3,0),)

     point1 = (0,3)
     point2 = (0.5,-1)
     point3 = (4, -1)

     print(computeTangentVectorToPolygon(p1, point1))

     print(computeTangentVectorToPolygon(p1, point2))

     print(computeTangentVectorToPolygon(p1, point3))


     return 0









if __name__ == "__main__":
     main()