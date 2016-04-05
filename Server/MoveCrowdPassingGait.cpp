#include "MoveCrowdPassingGait.h"
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

namespace CrowdPassing
{
const double CrowdPassingGaitWrapper::FORCE_LIMIT[6] = {120, 120, 120, 60, 60, 60};
const double CrowdPassingGaitWrapper::FORCE_DEADZONE[6] = {2, 2, 2, 2, 2, 2};
double CrowdPassingGaitWrapper::forceSum[6] = {0, 0, 0, 0, 0, 0};
double CrowdPassingGaitWrapper::forceBase[6] = {0, 0, 0, 0, 0, 0};
double CrowdPassingGaitWrapper::rawForce[6] = {0, 0, 0, 0, 0, 0};
double CrowdPassingGaitWrapper::mappedForce[6] = {0, 0, 0, 0, 0, 0};
double CrowdPassingGaitWrapper::filteredForce[6] = {0, 0, 0, 0, 0, 0};
double CrowdPassingGaitWrapper::feetPosition[18] = {0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0};
double CrowdPassingGaitWrapper::initialBodyPosition[6] = {0, 0, 0, 0, 0, 0};

int CrowdPassingGaitWrapper::zeroingCount = 0;
CrowdPassingPlanner CrowdPassingGaitWrapper::crowdPassingPlanner;
LowpassFilter<6> CrowdPassingGaitWrapper::lpf;
GAIT_CMD CrowdPassingGaitWrapper::command = GAIT_CMD::NOCMD;

Pipe<DiagnosticData> CrowdPassingGaitWrapper::dataPipe(true);
DiagnosticData CrowdPassingGaitWrapper::diagnosticData;

void CrowdPassingGaitWrapper::ParseCmds(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    CrowdPassingGaitParam param;

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

int CrowdPassingGaitWrapper::GaitFunction(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const CrowdPassingGaitParam &>(param_in);
    double timeNow = param.count * 0.001;

    switch (command)
    {
        case GAIT_CMD::NOCMD:
            break;
        case GAIT_CMD::INIT:
            if (crowdPassingPlanner.GetState() == CrowdPassingPlanner::VGS_STARTED ||
                crowdPassingPlanner.GetState() == CrowdPassingPlanner::VGS_STOPPING)
                break;
            lpf.Initialize();
            lpf.SetCutFrequency(0.03, 1000);
            crowdPassingPlanner.Initialize();
            rt_printf("GAIT CMD: INIT \n");
            break; 
        case GAIT_CMD::START:
            crowdPassingPlanner.Start(timeNow);
            rt_printf("GAIT CMD: START \n");
            break; 
        case GAIT_CMD::STOP:
            crowdPassingPlanner.RequireStop(timeNow);
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

    crowdPassingPlanner.DoIteration(timeNow, mappedForce, feetPosition);
    robot.SetPeb(initialBodyPosition);
    robot.SetPee(feetPosition, robot.ground());
    
    if (crowdPassingPlanner.GetState() == CrowdPassingPlanner::VGS_STOPPED)
        return 0;

    return 1;
}

void CrowdPassingGaitWrapper::RequireZeroing()
{
    zeroingCount = ZERO_SAMPLE_COUNT;
    for(int i = 0; i < 6; i++)
    {
        forceSum[i] = 0;
    }
}

void CrowdPassingGaitWrapper::Zeroing(const double* forceData)
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
void CrowdPassingGaitWrapper::ForceMapping(const double* srcForceData, double* destForceData)
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

void CrowdPassingGaitWrapper::StartReceiveData()
{
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
                        std::cout << std::setw(5) << std::fixed << std::setprecision(1) << data.forceData[j*3+i] << "  ";
                    }
                    std::cout << std::endl;
                }
            }

        });
}

}
