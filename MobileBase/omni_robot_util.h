#ifndef OMNI_ROBOT_UTIL
#define OMNI_ROBOT_UTIL

/*
    Defining length, width and height of the robot, and radius of the wheels.
*/

#define LENGTH 0.2
#define RADIUS 0.1
#define WIDTH  0.2
#define HEIGHT 0.1

void getAngVelFourWheels(double vx, double vy, double wz, double *u);
void getTwistFourWheels(double u1, double u2, double u3, double u4, double *twist);

#endif
