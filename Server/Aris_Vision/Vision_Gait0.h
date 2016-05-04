#ifndef VISION_GAIT0_H
#define VISION_GAIT0_H

#include <Robot_Gait.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace aris::dynamic;

enum robotMove
{
    nomove = 0,
    turn = 1,
    flatmove = 2,
    bodymove = 3,
    stepup = 4,
    stepdown = 5,
    avoidmove = 6,
};

struct VISION_WALK_PARAM
{
    int count = 0;
    robotMove movetype = nomove;
    int walkNum = 1;
    double walkDirection = M_PI;
    double walkLength = 0.5;
    int totalCount = 5000;
    double turndata = 0;
    double movedata[3] = {0, 0, 0};
    double bodymovedata[3] = {0, 0, 0};
    double stepupdata[6] = {0, 0, 0, 0, 0, 0};
    double stepdowndata[6] = {0, 0, 0, 0, 0, 0};
};

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param);

#endif // VISION_GAIT0_H
