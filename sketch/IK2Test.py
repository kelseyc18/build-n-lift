import math
wabh = 0 #wrist angle below horizontal
wristAng = 90
gripper = 100.00
ulna = 187.325
humerus = 146.05
base_height = 67.31
def calculateDegrees( x,  y,  z):
    alpha = wabh * math.pi/180.
    r = math.sqrt(x**2 + y**2)
    theta1 = math.atan2(y,x)
    length4 = math.sqrt((r-gripper*math.cos(alpha))**2 + (z - base_height + gripper*math.cos(alpha))**2)
    theta4 = math.atan2(z-base_height+gripper*math.sin(alpha),r-gripper*math.cos(alpha))
    theta5 = math.acos((ulna**2 - humerus**2 - length4**2)/(-2*humerus*length4))
    theta3 = math.acos((length4**2 - humerus**2 - ulna**2)/(-2*humerus*ulna))
    theta6 = math.pi - theta5 - theta3
    theta2 = theta5 + theta4
    wristfromulna = (theta6 + math.pi - theta4 - alpha)*180./math.pi
    theta1Deg = theta1/math.pi*180.
    theta2Deg = theta2/math.pi*180.
    theta3Deg = theta3/math.pi*180.
    if theta3Deg < 5:
        theta3Deg = 5
    wristAng = wristfromulna - 90
    print "Theta1",theta1Deg
    print "Theta2",theta2Deg
    print "Theta3",theta3Deg

    
