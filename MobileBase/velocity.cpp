#include <iostream>
#include "velocity.h"
using namespace std;

void calVelocity(int vx, int vy, int wz, double *u){
    int i;
    double A[4][3] = {{-LENGTH-WIDTH, 1, -1}, {LENGTH+WIDTH, 1, 1}, {LENGTH+WIDTH, 1, -1}, {-LENGTH-WIDTH, 1, 1}};
    double V[3]= {wz, vx, vy};
    // matrix calculation
    for (i = 0; i < 4; i++){
        u[i] = (A[i][0] * V[0] + A[i][1] * V[1] + A[i][2] *V[2])/RADIUS;
    }
}

int main()
{
    double a[4];
    calVelocity(0., 0., 1., a);
    cout << a[0]<<endl;
    cout << a[1]<<endl;
    cout << a[2]<<endl;
    Cout << a[3]<<endl;
    return 0;
}
