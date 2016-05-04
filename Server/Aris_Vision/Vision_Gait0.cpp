#include "Vision_Gait0.h"

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param)
{
    Robots::WalkParam wk_param;

    switch(param.movetype)
    {

    case avoidmove:
    {
        wk_param.alpha = param.walkDirection;
        wk_param.beta = 0;
        wk_param.d = param.walkLength;
        wk_param.h = 0.05;
        wk_param.n = param.walkNum;

        wk_param.count = param.count;
        wk_param.totalCount = param.totalCount;
    }
        break;

    case turn:
    {
        wk_param.alpha = 0;
        wk_param.beta = param.turndata*M_PI/180*2;
        wk_param.d = 0;
        wk_param.h = 0.05;

        wk_param.n = 1;
        wk_param.count = param.count;
        wk_param.totalCount = param.totalCount;
    }
        break;
    case flatmove:
    {
        if(param.movedata[0] != 0)
        {
            if(param.movedata[0] > 0)
            {
                wk_param.alpha = -M_PI/2;
                wk_param.d = param.movedata[0] * 2;
            }
            else
            {
                wk_param.alpha = M_PI/2;
                wk_param.d = -param.movedata[0] * 2;
            }
            wk_param.beta = 0;
            wk_param.h = 0.05;
        }
        else
        {
            wk_param.alpha = M_PI;
            wk_param.beta = 0;
            wk_param.d = param.movedata[2] * 2;
            wk_param.h = 0.05;
        }

        wk_param.n = 1;
        wk_param.count = param.count;
        wk_param.totalCount = param.totalCount;
    }
        break;
    default:
        break;
    }
    return Robots::walkGait(robot, wk_param);
}

