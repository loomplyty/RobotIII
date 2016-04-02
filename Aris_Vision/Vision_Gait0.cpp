#include "Vision_Gait0.h"
#include <string.h>
#include <math.h>
#include <iostream>

#ifndef PI
#define PI 3.141592653589793
#endif

using namespace std;

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param)
{
    Robots::WalkParam wk_param;

    switch(param.movetype)
    {
    case turn:
    {
        wk_param.alpha = 0;
        wk_param.beta = param.turndata*PI/180*2;
        wk_param.d = 0;
        wk_param.h = 0.05;
    }
        break;
    case flatmove:
    {
        if(param.movedata[0] != 0)
        {
            if(param.movedata[0] > 0)
            {
                wk_param.alpha = -PI/2;
                wk_param.d = param.movedata[0] * 2;
            }
            else
            {
                wk_param.alpha = PI/2;
                wk_param.d = -param.movedata[0] * 2;
            }
            wk_param.beta = 0;
            wk_param.h = 0.05;
        }
        else
        {
            wk_param.alpha = PI;
            wk_param.beta = 0;
            wk_param.d = param.movedata[2] * 2;
            wk_param.h = 0.05;
        }
    }
        break;
    default:
        break;
    }

    wk_param.n = 1;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;

    return Robots::walkGait(robot, wk_param);
}

void RobotBody(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam)
{
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double beginPeb[6];

    if (pParam.count%pParam.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.GetPeb(beginPeb, beginMak);
    }

    double Peb[6], Pee[18];
    std::copy(beginPeb, beginPeb + 6, Peb);
    std::copy(beginPee, beginPee + 18, Pee);

    double s = -(PI / 2)*cos(PI * (pParam.count + 1) / pParam.totalCount) + PI / 2;

    Peb[0] += pParam.bodymovedata[0] * (1 - cos(s))/2;
    Peb[1] += pParam.bodymovedata[1] * (1 - cos(s))/2;
    Peb[2] += pParam.bodymovedata[2] * (1 - cos(s))/2;

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);
}

void RobotStepUp(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam)
{
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (pParam.count%pParam.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double pBodyPE[6] = {0, 0, 0, 0, 0, 0};

    double pEE[18] =
    { -0.3, -1.05, -0.65,
      -0.45, -1.05, 0,
      -0.3, -1.05, 0.65,
      0.3, -1.05, -0.65,
      0.45, -1.05, 0,
      0.3, -1.05, 0.65 };

    double stepUpH = 0.25;
    double stepUpD = 0.325;

    double StepUpNextPos[6] = {0, 0, 0, 0, 0, 0};

    memcpy(StepUpNextPos,pParam.stepupdata,6*sizeof(double));

    static double StepUpCurrentPos[6] = {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05};

    for(int i = 0; i < 6; i++)
    {
        pEE[i*3 + 1] = StepUpCurrentPos[i];
    }

    int periodcounter = pParam.totalCount / 6;

    if(pParam.count < periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05)) * (1 - cos(s))/2;
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05)) * (1 - cos(s))/2;
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05)) * (1 - cos(s))/2;
    }
    else if(pParam.count >= periodcounter && pParam.count < 2 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05));

        pEE[2] += stepUpD * (1 - cos(s))/2;
        pEE[8] += stepUpD * (1 - cos(s))/2;
        pEE[14] += stepUpD * (1 - cos(s))/2;

        pBodyPE[2] += stepUpD/2 * (1 - cos(s))/2;
    }
    else if(pParam.count >= 2 * periodcounter && pParam.count < 3 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05)) * (1 - cos(s)) / 2;
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05)) * (1 - cos(s)) / 2;
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05)) * (1 - cos(s)) / 2;

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2;
    }
    else if(pParam.count >= 3 * periodcounter && pParam.count < 4 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05)) * (1 - cos(s))/2;
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05)) * (1 - cos(s))/2;
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05)) * (1 - cos(s))/2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2;
    }
    else if(pParam.count >= 4 * periodcounter && pParam.count < 5 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05));
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05));
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05));

        pEE[5] += stepUpD * (1 -cos(s))/2;
        pEE[11] += stepUpD * (1 -cos(s))/2;
        pEE[17] += stepUpD * (1 -cos(s))/2;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2 + stepUpD/2 * (1 - cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepUpH - (StepUpCurrentPos[1] + 1.05))
                - (stepUpH - (StepUpNextPos[1] + 1.05)) * (1 - cos(s))/2;
        pEE[10] += (stepUpH - (StepUpCurrentPos[3] + 1.05))
                - (stepUpH - (StepUpNextPos[3] + 1.05)) * (1 - cos(s))/ 2;
        pEE[16] += (stepUpH - (StepUpCurrentPos[5] + 1.05))
                - (stepUpH - (StepUpNextPos[5] + 1.05)) * (1 - cos(s))/2;

        pEE[5] += stepUpD;
        pEE[11] += stepUpD;
        pEE[17] += stepUpD;

        pEE[1] += (stepUpH - (StepUpCurrentPos[0] + 1.05))
                - (stepUpH - (StepUpNextPos[0] + 1.05));
        pEE[7] += (stepUpH - (StepUpCurrentPos[2] + 1.05))
                - (stepUpH - (StepUpNextPos[2] + 1.05));
        pEE[13] += (stepUpH - (StepUpCurrentPos[4] + 1.05))
                - (stepUpH - (StepUpNextPos[4] + 1.05));

        pEE[2] += stepUpD;
        pEE[8] += stepUpD;
        pEE[14] += stepUpD;

        pBodyPE[2] += stepUpD/2 + stepUpD/2;
    }

    if (pParam.totalCount - pParam.count - 1 == 0)
    {
        memcpy(StepUpCurrentPos, StepUpNextPos, 6*sizeof(double));
    }

    robot.SetPeb(pBodyPE, beginMak);
    robot.SetPee(pEE, beginMak);
}

void RobotStepDown(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam)
{
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (pParam.count%pParam.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double pBodyPE[6] = {0, 0, 0, 0, 0, 0};

    double pEE[18] =
    { -0.3, -0.85, -0.65,
      -0.45, -0.85, 0,
      -0.3, -0.85, 0.65,
      0.3, -0.85, -0.65,
      0.45, -0.85, 0,
      0.3, -0.85, 0.65 };

    static double StepDownCurrentPos[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};

    for(int i = 0; i < 6; i++)
    {
        pEE[i*3 + 1] = StepDownCurrentPos[i];
    }

    double stepDownH = 0.05;
    double stepDownD = 0.325;

    double StepDownNextPos[6] = {0, 0, 0, 0, 0, 0};

    memcpy(StepDownNextPos,pParam.stepdowndata,6*sizeof(double));

    int periodcounter = pParam.totalCount / 6;

    if(pParam.count < periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85)) * (1 - cos(s))/2;
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85)) * (1 - cos(s))/2;
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85)) * (1 - cos(s))/2;
    }
    else if(pParam.count >= periodcounter && pParam.count < 2 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85));

        pEE[2] += stepDownD * (1 -cos(s))/2;
        pEE[8] += stepDownD * (1 -cos(s))/2;
        pEE[14] += stepDownD * (1 -cos(s))/2;

        pBodyPE[2] += stepDownD/2 * (1 - cos(s))/2;
    }
    else if(pParam.count >= 2 * periodcounter && pParam.count < 3 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85)) * (1 - cos(s)) / 2;
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85)) * (1 - cos(s)) / 2;
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85)) * (1 - cos(s)) / 2;

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2;
    }
    else if(pParam.count >= 3 * periodcounter && pParam.count < 4 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85)) * (1 - cos(s))/2;
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85)) * (1 - cos(s))/2;
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85)) * (1 - cos(s))/2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2;
    }
    else if(pParam.count >= 4 * periodcounter && pParam.count < 5 * periodcounter)
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85));
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85));
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85));

        pEE[5] += stepDownD * (1 -cos(s))/2;
        pEE[11] += stepDownD * (1 -cos(s))/2;
        pEE[17] += stepDownD * (1 -cos(s))/2;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2 + stepDownD/2 * (1 - cos(s))/2;
    }
    else
    {
        double s = -(PI / 2)*cos(PI * (pParam.count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85))
                - (stepDownH + (-StepDownNextPos[1] - 0.85)) * (1 - cos(s)) / 2;
        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85))
                - (stepDownH + (-StepDownNextPos[3] - 0.85)) * (1 - cos(s)) / 2;
        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85))
                - (stepDownH + (-StepDownNextPos[5] - 0.85)) * (1 - cos(s)) / 2;

        pEE[5] += stepDownD;
        pEE[11] += stepDownD;
        pEE[17] += stepDownD;

        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
                - (stepDownH + (-StepDownNextPos[0] - 0.85));
        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
                - (stepDownH + (-StepDownNextPos[2] - 0.85));
        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
                - (stepDownH + (-StepDownNextPos[4] - 0.85));

        pEE[2] += stepDownD;
        pEE[8] += stepDownD;
        pEE[14] += stepDownD;

        pBodyPE[2] += stepDownD/2 + stepDownD/2;
    }

    if (pParam.totalCount - pParam.count - 1 == 0)
    {
        memcpy(StepDownCurrentPos, StepDownNextPos, 6*sizeof(double));
    }

    robot.SetPeb(pBodyPE, beginMak);
    robot.SetPee(pEE, beginMak);
}

