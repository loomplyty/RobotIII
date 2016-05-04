#ifndef ROFO2_H
#define ROFO2_H

#include <aris.h>
//#include "Aris_Control.h"
//#include <Aris_Message.h>
//#include <Aris_Core.h>
#include "StaMach.h"
#include "Gait.h"
#include <Robot_Base.h>
#include <Robot_Type_I.h>
#include "Compatible.h"
//#include "RofoConfig.h"


using namespace aris::core;

extern force_gait::GaitRobot rofo;

extern bool IsRofoEnd;

extern LogData temp_log_data;


namespace Rofo {




struct CLIMB_PARAM: public aris::server::GaitParamBase
{
    int count;
};

//struct DIG_PARAM: public Aris::Server::GaitParamBase
//{
//    double depth;
//};

struct Ay_Param: public aris::server::GaitParamBase
{
    double aY; // real value of y, -1.1<aY<-0.72
    double dY;
    bool IsRel=false;
    std::int32_t ay_count{5000};
    //same as RecoverParam
    std::int32_t recover_count{ 3000 };
    std::int32_t align_count{ 3000 };
    bool active_leg[6]{ true,true,true,true,true,true };
    double margin_offset{0.01};//meter
//    double alignPee[18]
//    { -0.3,   -0.75,   -0.65,
//        -0.45,  -0.75,   0,
//        -0.3,   -0.75,    0.65,
//        0.3,   -0.75,    -0.65,
//        0.45,  -0.75,    0,
//        0.3,   -0.75,     0.65 };
//    double recoverPee[18]
//    { -0.3,   -0.85,   -0.65,
//        -0.45,  -0.85,   0,
//        -0.3,   -0.85,    0.65,
//        0.3,   -0.85,    -0.65,
//        0.45,  -0.85,    0,
//        0.3,   -0.85,     0.65 };

};

//Aris::Core::MSG parseClimb(const std::string &cmd, const map<std::string, std::string> &params);
//Aris::Core::MSG parseEndClimb(const std::string &cmd, const map<std::string, std::string> &params);
//Aris::Core::MSG parseAdjust2Climb(const std::string &cmd, const map<std::string, std::string> &params);
//Aris::Core::MSG parseAdjustY(const std::string &cmd, const map<std::string, std::string> &params);

//Aris::Core::MSG parseDig(const std::string &cmd, const map<std::string, std::string> &params);

//int Dig(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param_in);


//int adjustY(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param_in);

int RofoWalkInit();

auto rofoParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto rofoGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

auto rofoEndParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto rofoEndGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

auto ayParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto ayGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

}
#endif
