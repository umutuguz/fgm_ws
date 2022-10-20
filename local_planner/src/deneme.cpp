#include <iostream>
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <vector>

using namespace std;

int main()
{
    double phiGoal, odomRX, odomRY,goalX,goalY;

    odomRX = -1;
    odomRY = 1;
    goalX = -5;
    goalY = 3;

    phiGoal = atan2(goalY - odomRY, goalX - odomRX);

    if (odomRX < goalX)
        phiGoal = (M_PI/2) - phiGoal;

    if (odomRX > goalX)
        phiGoal = (M_PI*2) - phiGoal;

    
    cout << "phigoal is : " << phiGoal;
    cout << "phigoal is : " << phiGoal *180 / 3.1415;

    return 0;
}