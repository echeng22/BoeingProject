#IFNDEF OMNI_ROBOT_UTIL
#DEFINE OMNI_ROBOT_UTIL

void getAngVelFourWheels(double radius, double length, double width, double vx, double vy, double wz, double *u);
void getTwistFourWheels(double radius, double length, double width, double u1, double u2, double u3, double u4, double *twist);

#ENDIF
