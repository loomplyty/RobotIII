#ifndef CONTINUOUS_MOVE_H
#define CONTINUOUS_MOVE_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct ContinuousMoveParam final :public aris::server::GaitParamBase
{
    std::int32_t move_direction;
};

struct CM_RecordParam
{
    double bodyPE_last[6];
    double bodyVel_last[6];

    double forceSum[6]{0,0,0,0,0,0};
    double forceAvg[6]{0,0,0,0,0,0};
    double force[6];
};

/*parse function*/
void parseContinuousMoveBegin(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseContinuousMoveJudge(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

/*operation function*/
int continuousMove(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);


#endif // CONTINUOUS_MOVE_H
