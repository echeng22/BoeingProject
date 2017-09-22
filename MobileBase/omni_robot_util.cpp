#include "omni_robot_util.h"

void getAngVelFourWheels(double vx, double vy, double wz, double *u)
{
    int i;
    double H[4][3] = {{-LENGTH - WIDTH, 1, -1}, {LENGTH + WIDTH, 1, 1}, {LENGTH + WIDTH, 1, -1}, {-LENGTH - WIDTH, 1, 1}};
    double twist[3] = {wz, vx, vy};
    // matrix calculation
    for (i = 0; i < 4; i++)
    {
        u[i] = (H[i][0] * twist[0] + H[i][1] * twist[1] + H[i][2] * twist[2]) / RADIUS;
    }
}

void getTwistFourWheels(double u1, double u2, double u3, double u4, double *twist)
{
    double Vx = (u1 + u2) * RADIUS / 2;
    double Vy = RADIUS / 2 * (u2 + u4 - 2 * (Vx / RADIUS));
    double Wz = (u1 - (Vx / RADIUS) + (Vy / RADIUS)) * RADIUS / (-LENGTH - WIDTH);
    twist[0] = Vx;
    twist[1] = Vy;
    twist[2] = Wz;
}
