#include "Vision_AvoidControl.h"

void DistanceComputation(Pose cRobotPos, ObstaclePosition cObstaclePos, double cdistance[2])
{
    double l = 0.275;
    double robRadius = sqrt(l*l + 0.45*0.45);

    double frontCenterX = l*cos(M_PI/2 + cRobotPos.gama) + cRobotPos.X;
    double frontCenterY = l*sin(M_PI/2 + cRobotPos.gama) + cRobotPos.Y;

    double backCenterX = l*cos(3*M_PI/2 + cRobotPos.gama) + cRobotPos.X;
    double backCenterY = l*sin(3*M_PI/2 + cRobotPos.gama) + cRobotPos.Y;

    double frontDistance = sqrt(pow((frontCenterX - cObstaclePos.X),2) + pow((frontCenterY - cObstaclePos.Y),2)) - robRadius - cObstaclePos.radius;
    double backDistance = sqrt(pow((backCenterX - cObstaclePos.X),2) + pow((backCenterY - cObstaclePos.Y),2)) - robRadius - cObstaclePos.radius;

    cdistance[0] = frontDistance;
    cdistance[1] = backDistance;
}

double PenaltyParameter(Pose cRobotPos, ObstaclePosition cObstaclePos)
{
    double distance[2]{0, 0};
    double cPenaltyValue[2]{0, 0};

    DistanceComputation(cRobotPos, cObstaclePos, distance);

    for(int i = 0; i < 2; i++)
    {
        if(distance[i] <= 0.05)
        {
            cPenaltyValue[i] = 1;
        }
        else if(distance[i] >= 0.05&&distance[i] <= 0.2)
        {
            cPenaltyValue[i] = 0.5*(1 + tanh(1/(distance[i] - 0.05) + 1/(distance[i] - 0.2)));
        }
        else
        {
            cPenaltyValue[i] = 0;
        }
    }
    double penaltyValue = cPenaltyValue[0] + cPenaltyValue[1];
    return penaltyValue;
}

void AvoidControl::AvoidWalkControl(Pose cTargetPos, Pose cRobotPos, vector<ObstaclePosition> cObstaclePoses)
{
    memset(&avoidWalkParam, 0, sizeof(avoidWalkParam));
    double robotWalkDirection[14];
    double robotWalkVector[14][2];
    double robotwalkStepNum = 2;

    double robotWalkStepLength[14] = {0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.45, 0.40, 0.35, 0.30, 0.25, 0.20, 0};

    int directionLabel = 13;

    for(int i = 0; i < 13; i++)
    {
        robotWalkDirection[i] = i*15*M_PI/180;
    }

    if((cTargetPos.X - cRobotPos.X) < 0.5)
    {
          robotWalkDirection[13] = M_PI/2;
    }
    else
    {
        robotWalkDirection[13] = atan((cTargetPos.Y - cRobotPos.Y)/(cTargetPos.X - cRobotPos.X));
    }
    for(int i = 0; i < 14; i++)
    {
        robotWalkVector[i][0] = 1*cos(robotWalkDirection[i]);
        robotWalkVector[i][1] = 1*sin(robotWalkDirection[i]);
    }

    robotWalkStepLength[13] = 0.50 - fabs(robotWalkDirection[13] - M_PI/2)*0.3/(M_PI/2);

    double minCost = 1000;
    double costFunction[14]{0};

    for(int i = 0; i < 14; i++)
    {
        Pose tempRobotPos = cRobotPos;
        costFunction[i] = 1/(1 + robotWalkVector[i][0]*robotWalkVector[13][0] + robotWalkVector[i][1]*robotWalkVector[13][1]);

        for(int j = 0; j < robotwalkStepNum; j++)
        {
            if(robotwalkStepNum == 1 || robotwalkStepNum == 2)
            {
                tempRobotPos.X = tempRobotPos.X + 0.5*robotWalkStepLength[i]*robotWalkVector[i][0];
                tempRobotPos.Y = tempRobotPos.Y + 0.5*robotWalkStepLength[i]*robotWalkVector[i][1];
            }
            else
            {
                if(j == 0)
                {
                    tempRobotPos.X = tempRobotPos.X + 0.5*robotWalkStepLength[i]*robotWalkVector[i][0];
                    tempRobotPos.Y = tempRobotPos.Y + 0.5*robotWalkStepLength[i]*robotWalkVector[i][1];
                }
                else if(j < robotwalkStepNum - 1)
                {
                    tempRobotPos.X = tempRobotPos.X + robotWalkStepLength[i]*robotWalkVector[i][0];
                    tempRobotPos.Y = tempRobotPos.Y + robotWalkStepLength[i]*robotWalkVector[i][1];
                }
                else
                {
                    tempRobotPos.X = tempRobotPos.X + 0.5*robotWalkStepLength[i]*robotWalkVector[i][0];
                    tempRobotPos.Y = tempRobotPos.Y + 0.5*robotWalkStepLength[i]*robotWalkVector[i][1];
                }
            }
            if (cObstaclePoses.size() > 0)
            {
                for(vector<ObstaclePosition>::iterator obsIter = cObstaclePoses.begin(); obsIter != cObstaclePoses.end(); obsIter++ )
                {
                    costFunction[i] = costFunction[i] + PenaltyParameter(tempRobotPos, *obsIter);
                }
            }
            else
            {
                costFunction[i] = costFunction[i] + 0;
            }
        }

        if(minCost > costFunction[i])
        {
            minCost = costFunction[i];
            directionLabel = i;
            nextRobotPos = tempRobotPos;
        }
    }
    cout<<"Direction Num: ------"<<directionLabel<<endl;
    avoidWalkParam.stepLength = robotWalkStepLength[directionLabel];
    avoidWalkParam.stepNum = robotwalkStepNum;
    avoidWalkParam.walkDirection = robotWalkDirection[directionLabel] + M_PI/2;
}
