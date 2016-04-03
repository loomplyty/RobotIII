#ifndef CONTINUOUS_WALK_WITH_FORCE_H
#define CONTINUOUS_WALK_WITH_FORCE_H

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

class CwfState
{
public:
    static CwfState& getState()
    {
        static CwfState s;
        return s;
    }
    bool& isStopping() { return isStopping_; }
private:
    bool isStopping_{ true };
    CwfState() = default;
};

enum WALK_DIRECTION
{
    STOP,
    FORWARD,
    BACKWARD,
    RIGHTWARD,
    LEFTWARD,
    TURNLEFT,
    TURNRIGHT,
    FAST_TURNLEFT,
    FAST_TURNRIGHT
};

/*parse function*/
auto CWFParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto CWFStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;

/*operation function*/
auto CWFGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
WALK_DIRECTION forceJudge(const double *force, const double *threshold);

#endif // CONTINUOUS_WALK_WITH_FORCE_H
