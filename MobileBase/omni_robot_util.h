#include <iostream>
#include "omni_robot_util.h"

void getAngVelFourWheels(double radius, double length, double width, double vx, double vy, double wz, double *u)
{
    int i;
    double H[4][3] = {{-length - width, 1, -1}, {length + width, 1, 1}, {length + width, 1, -1}, {-length - width, 1, 1}};
    double twist[3] = {wz, vx, vy};
    // matrix calculation
    for (i = 0; i < 4; i++)
    {
        u[i] = (H[i][0] * twist[0] + H[i][1] * twist[1] + H[i][2] * twist[2]) / radius;
    }
}

void getTwistFourWheels(double radius, double length, double width, double u1, double u2, double u3, double u4, double *twist)
{
    double Vx = (u1 + u2) * radius / 2;
    double Vy = radius / 2 * (u2 + u4 - 2 * (Vx / radius));
    double Wz = (u1 - (Vx / radius) + (Vy / radius)) * radius / (-length - width);
    twist[0] = Vx;
    twist[1] = Vy;
    twist[2] = Wz;
}

// int main()
// {
//     // add(2, 3);
//     double u[4];
//     getAngVelFourWheels(0.25, 1.3, 1.27, 1, 1.267, -1.24, u);
//     double twist[3];
//     getTwistFourWheels(0.25, 1.3, 1.27, u[0], u[1], u[2], u[3], twist);
//     std::cout << twist[0] << ", " << twist[1] << ", " << twist[2] << std::endl;

//     return 0;
// }
