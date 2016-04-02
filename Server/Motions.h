#ifndef MOTIONS_H
#define MOTIONS_H

//#include <aris_control_pipe.h>
#include <aris.h>


#include <thread>
#include <functional>
#include <cstdint>
#include <map>
#include <Robot_Base.h>
#include <Robot_Gait.h>

using namespace std;



namespace GoStair
{

struct GoStairParam final:public aris::server::GaitParamBase
{
    //const char fileName[256]{ 0 };
    std::int32_t gaitCount{ 0 };

    // double *pIn{ nullptr };
};


void parseGoStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);

int GoUpStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

}


#endif
