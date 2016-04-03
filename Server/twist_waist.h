#ifndef TWIST_WAIST_H
#define TWIST_WAIST_H

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

struct twParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 9000 };
    double pitchMax{ 0.2 };
    double rollMax{ 0.2 };
    double diameter{ 0.1 };
    double height{ 0.05 };
};

auto twistWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto twistWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // TWIST_WAIST_H
