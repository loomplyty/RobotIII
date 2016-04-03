#ifndef PUSH_RECOVERY_H
#define PUSH_RECOVERY_H

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

#include "interpolation.h"

#ifndef PI
#define PI 3.141592653589793
#endif

class PrState
{
public:
    static PrState& getState()
    {
        static PrState s;
        return s;
    }
    bool& isStopping() { return isStopping_; }
private:
    bool isStopping_{ true };
    PrState() = default;
};

/*gait parameters*/
struct prParam final:public aris::server::GaitParamBase
{
    std::int32_t pushCount{1000};
    std::int32_t recoverCount{4000};
    std::int32_t totalCount{5000};
    std::int32_t firstStepCount{2000};
    double d{0.3};//步长
    double h{0.05};//步高
    double angle{5};//身体最大摆角
    double descend{0.025};//身体下降高度
};

/*parse function*/
auto pushRecoveryParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto pushRecoveryStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;

/*operation function*/
auto pushRecoveryGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // PUSH_RECOVERY_H
