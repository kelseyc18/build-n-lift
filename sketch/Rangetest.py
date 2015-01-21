import math
base = 67.31
humerus = 146.05
ulna = 187.325
gripper = 100
bad_points = []

import numpy
for x in range(200,400,5):
    for y in range(0,25,5):
        for z in range(0,25,25):
            for wrist in range(16,17,1):
                phi = wrist*10.0 +1.0
                phi = phi*math.pi/180
                r = math.sqrt(x**2 + y**2)
                theta1 = math.atan2(y,x)
                length5 = math.sqrt(ulna**2+gripper**2 - 2*ulna*gripper*math.cos(math.pi-phi))
                cphi2 = (gripper**2 - length5**2 -ulna**2)/(-2*length5*ulna)
                length4 = math.sqrt(r**2 + (z-base)**2)
                ctheta3 = (length4**2 - length5**2 - ulna**2)/(-2*length5*ulna)
                theta4 = math.atan2(z-base,r)
                ctheta5 = (length5**2 - humerus**2 - length4**2)/(-2*humerus*length4)
                if math.fabs(cphi2) > 1 or math.fabs(ctheta3) > 1 or math.fabs(ctheta5) > 1:
                    bad_points.append([x,y,z,wrist*10])

print bad_points
                
                
# 215, 215, 
