#include "MovePushDoorSimple.h"
#include "rtdk.h"

#ifdef UNIX
#include "rtdk.h"
#endif
#ifdef WIN32
#define rt_printf printf
#endif

#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

namespace PushDoorSimple
{
const double PushDoorSimpleWrapper::FORCE_LIMIT[6] = {120, 120, 120, 60, 60, 60};
const double PushDoorSimpleWrapper::FORCE_DEADZONE[6] = {2, 2, 2, 2, 2, 2};
double PushDoorSimpleWrapper::forceSum[6] = {0, 0, 0, 0, 0, 0};
double PushDoorSimpleWrapper::forceBase[6] = {0, 0, 0, 0, 0, 0};
double PushDoorSimpleWrapper::rawForce[6] = {0, 0, 0, 0, 0, 0};
double PushDoorSimpleWrapper::mappedForce[6] = {0, 0, 0, 0, 0, 0};
double PushDoorSimpleWrapper::filteredForce[6] = {0, 0, 0, 0, 0, 0};
double PushDoorSimpleWrapper::feetPosition[18] = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
double PushDoorSimpleWrapper::initialBodyPosition[6] = {0, 0, 0, 0, 0, 0};

int PushDoorSimpleWrapper::zeroingCount = 0;
PushDoorSimplePlanner PushDoorSimpleWrapper::pushDoorSimplePlanner;
LowpassFilter<6> PushDoorSimpleWrapper::lpf;
GAIT_CMD PushDoorSimpleWrapper::command = GAIT_CMD::NOCMD;

Pipe<DiagnosticData> PushDoorSimpleWrapper::dataPipe(true);
DiagnosticData PushDoorSimpleWrapper::diagnosticData;

void PushDoorSimpleWrapper::ParseCmds(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    PushDoorSimpleParam param;

    for(auto &i:params)
    {

        if(i.first=="i")
        {
            command = GAIT_CMD::INIT;
            msg.copyStruct(param);
            break;
        }
        else if(i.first=="b")
        {
            command = GAIT_CMD::START;
            break;
        }
        else if(i.first=="e")
        {
            command = GAIT_CMD::STOP;
            break;
        }
        else if(i.first=="c")
        {
            command = GAIT_CMD::CLEAR_FORCE;
            break;
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    std::cout<<"finished parse"<<std::endl;
}

int PushDoorSimpleWrapper::GaitFunction(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const PushDoorSimpleParam &>(param_in);
    double timeNow = param.count * 0.001;

    switch (command)
    {
        case GAIT_CMD::NOCMD:
            break;
        case GAIT_CMD::INIT:
            lpf.Initialize();
            lpf.SetCutFrequency(0.03, 1000);
            pushDoorSimplePlanner.Initialize();
            rt_printf("GAIT CMD: INIT \n");
            break; 
        case GAIT_CMD::START:
            pushDoorSimplePlanner.Start(timeNow);
            rt_printf("GAIT CMD: START \n");
            break; 
        case GAIT_CMD::STOP:
            pushDoorSimplePlanner.RequireStop(timeNow);
            rt_printf("GAIT CMD: STOP \n");
            break; 
        case GAIT_CMD::CLEAR_FORCE:
            rt_printf("GAIT CMD: CLEAR \n");
            RequireZeroing();
            break; 
    }
    
    command = GAIT_CMD::NOCMD;

    if (param.count % 500 == 0)
    {
        for(int i = 0; i < 6; i++)
        {
            diagnosticData.forceData[i] = rawForce[i];
        }
        dataPipe.sendToNrt(diagnosticData);
    }

    // Do zeroing if required
    Zeroing(param.force_data->at(0).fce);
    for(int i = 0; i < 6; i++)
    {
        rawForce[i] = param.force_data->at(0).fce[i] - forceBase[i];
    }
    lpf.DoFilter(rawForce, filteredForce);
    ForceMapping(filteredForce, mappedForce);

    pushDoorSimplePlanner.DoIteration(timeNow, mappedForce, feetPosition);
    robot.SetPeb(initialBodyPosition);
    robot.SetPee(feetPosition, robot.ground());
    
    if (pushDoorSimplePlanner.GetState() == PushDoorSimplePlanner::VGS_STOPPED)
        return 0;

    return 1;
}

void PushDoorSimpleWrapper::RequireZeroing()
{
    zeroingCount = ZERO_SAMPLE_COUNT;
    for(int i = 0; i < 6; i++)
    {
        forceSum[i] = 0;
    }
}

void PushDoorSimpleWrapper::Zeroing(const double* forceData)
{
    if (zeroingCount > 0)
    {
        for(int i = 0; i < 6; i++)
        {
            forceSum[i] += forceData[i];
        }

        if (zeroingCount == 1)
        {
            for(int i = 0; i < 6; i++)
            {
                forceBase[i] = forceSum[i]/ZERO_SAMPLE_COUNT;
            }
        }

        zeroingCount--;
    }
}
void PushDoorSimpleWrapper::ForceMapping(const double* srcForceData, double* destForceData)
{
    // map sensed force to model
    destForceData[0] =-1 * srcForceData[1];
    destForceData[1] = 1 * srcForceData[0] ;
    destForceData[2] = 1 * srcForceData[2] ;
    destForceData[3] =-1 * srcForceData[4] ;
    destForceData[4] = 1 * srcForceData[3] ;
    destForceData[5] = 1 * srcForceData[5] ;

    // saturation for force
    for(int i = 0; i < 6; i++)
    {
        if (fabs(destForceData[i]) < 2.0)
            destForceData[i] = 0;
        if (destForceData[i] > FORCE_LIMIT[i])
            destForceData[i] = FORCE_LIMIT[i];
        if (destForceData[i] < -FORCE_LIMIT[i])
            destForceData[i] = -FORCE_LIMIT[i];
    }
}

void PushDoorSimpleWrapper::StartReceiveData()
{
    using namespace std;
    static std::thread dataReceivingThread = std::thread([&]()
        {
            struct DiagnosticData data;

            long long count = -1;
            while (1)
            {
                dataPipe.recvInNrt(data);

                std::cout << ++count << " ";

                for (int j = 0; j < 2; j++)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        std::cout << setw(5) << std::fixed << setprecision(1) << data.forceData[j*3+i] << "  ";
                    }
                    std::cout << std::endl;
                }
            }

        });
}

}
