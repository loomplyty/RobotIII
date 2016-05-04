#ifndef VISION_AVOIDCONTROL_H
#define VISION_AVOIDCONTROL_H

#include "Vision_ObstacleDetection.h"
#include <math.h>

struct SimpleWalkParam
{
    int stepNum = 1;
    double stepLength = 0.5;
    double walkDirection = M_PI;
};

class AvoidControl
{
public:
    SimpleWalkParam avoidWalkParam;
    AvoidControl(){}
    ~AvoidControl(){}
    Pose nextRobotPos;
    void AvoidWalkControl(Pose cTargetPos, Pose cRobotPos, vector<ObstaclePosition> cObstaclePoss);
};

#endif // VISION_AVOIDCONTROL_H
