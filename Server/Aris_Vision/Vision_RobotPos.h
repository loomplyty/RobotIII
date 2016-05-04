#ifndef VISION_ROBOTPOS_H
#define VISION_ROBOTPOS_H

#include <iostream>
#include <vector>

using namespace std;

struct Pose
{
    double X;
    double Y;
    double Z;
    double alpha;
    double beta;
    double gama;
};

class RobotPose
{
public:
    RobotPose();
    ~RobotPose();
    vector<Pose> robotPoses;
};

#endif // VISION_ROBOTPOS_H
