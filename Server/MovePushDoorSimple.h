#ifndef MOVE_PUSH_DOOR_SIMPLE_H 
#define MOVE_PUSH_DOOR_SIMPLE_H

#include <thread>
#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <aris_control_pipe.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>
#include "PushDoorSimple.h"
#include "LowpassFilter.h"

using namespace aris::control;

namespace PushDoorSimple
{
    enum class GAIT_CMD
    {
        NOCMD = 0,
        INIT  = 1,
        START = 2,
        STOP  = 3,
        CLEAR_FORCE = 4
    };

    struct PushDoorSimpleParam final : public aris::server::GaitParamBase
    {
    };

    struct DiagnosticData
    {
        double forceData[6];
    };

    class PushDoorSimpleWrapper
    {
        public:
            static const double FORCE_DEADZONE[6];
            static const double FORCE_LIMIT[6];
            static const int ZERO_SAMPLE_COUNT = 500;
            static int  zeroingCount;
            static double rawForce[6];
            static double forceSum[6];
            static double forceBase[6];
            static double filteredForce[6];
            static double mappedForce[6];
            static double feetPosition[18];
            static double initialBodyPosition[6];
            static PushDoorSimplePlanner pushDoorSimplePlanner;
            static LowpassFilter<6> lpf;
            static GAIT_CMD command;
            static Pipe<DiagnosticData> dataPipe;
            static DiagnosticData diagnosticData;
            
            static void ParseCmds(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
            static int GaitFunction(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
            static void StartReceiveData();
        private:
            static void RequireZeroing();
            static void Zeroing(const double* forceData);
            static void ForceMapping(const double* srcForceData, double* destForceData);
    };

    static PushDoorSimpleWrapper wrapper;
}

#endif // MOVE_GAIT_H
