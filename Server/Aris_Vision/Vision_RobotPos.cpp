#include "Vision_RobotPos.h"

RobotPose::RobotPose()
{
    Pose robotInitPos = {0, 0, 0, 0, 0, 0};
    robotPoses.push_back(robotInitPos);
}

RobotPose::~RobotPose()
{
    ;
}
