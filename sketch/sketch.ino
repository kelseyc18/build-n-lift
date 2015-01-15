#include<math.h>

/*
Daniel's Inverse Kinematics math
*/
const float pi = 3.14159265259
int x = 2; // x we give
int y = 2; // y we give
int z = 2; // z we give
float phi = 2; // wrist angle we give, between end effector and line of forearm
float length1 = 1; // height from base to shoulder
float length2 = 1; // length from shoulder to elbow
float length3 = 1; // length from elbow to wrist
float ro = 1; //length from wrist to end effector
float r = 0; // distance to end effector from base on base plane
float theta1 = 0;  // base angle
float theta2 = 0; // angle of the servo at the shoulder, from horizontal
float theta3 = 0; // angle of the servo at the elbow, between wrist and shoulder
r = sqrt(x^2 +y^2); // pythagoras
theta1 = atan2(y,x); // trig
float length5 = sqrt(length3^2 + ro^2 - 2*length3*ro*cos(180-phi)); // law of cosines to find the length from the wrist to the end effector
float phi2 = acos((-ro^2 - length5^2 + length3^2)/(-2*length5*length3)); //angle from wrist-elbow-end effector
float length4 = sqrt(r^2 + (z- length1)^2); // length from shoulder to end effector
theta3 = acos((length4^2 - length5^2 - length2^2)/(-2*length5*length2) + phi2; // elbow angle using law of cosines, correcting for the fact that the end effector placement angle is not the same as the elbow angle due to phiand ro being nonzero.
float theta4 = atan2(z-length1, r); // angle from horizontal to end effector
float theta5 = acos((length5^2 - length2^2 - length4^2)/(-2*length2*length4); // angle from theta4 to humerus
theta2 =theta4 +theta5; // adding to get theta 2

