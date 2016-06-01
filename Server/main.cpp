#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>
#include <Aris_Vision.h>
#include "Vision_Terrain0.h"
#include "Vision_Control0.h"
#include "Vision_Gait0.h"
#include "Motions.h"

#include "rtdk.h"
#include "unistd.h"

//liujimu's gaits
#include "continuous_walk_with_force.h"
#include "push_recovery.h"
#include "continuous_move.h"
#include "twist_waist.h"
#include "say_hello.h"
#include "peg_in_hole.h"
//zhaoyue
#include "swing.h"
//sunqiao
#include "MovePushDoorSimple.h"
#include "MoveCrowdPassingGait.h"
#include "rofo2.h"
using namespace aris::core;

aris::sensor::KINECT kinect1;

TerrainAnalysis terrainAnalysisResult;

atomic_bool isTerrainAnalysisFinished(false);

atomic_bool isSending(false);
atomic_bool isStop(false);

VISION_WALK_PARAM visionWalkParam;

struct matrix44
{
    double pm[4][4];
};

aris::control::Pipe<int> visionPipe(true);
aris::control::Pipe<matrix44> visionRecordPipe(true);

static auto visionRecordThread = std::thread([]()
{
    while(true)
    {
        matrix44 currentPm;

        visionRecordPipe.recvInNrt(currentPm);

        auto visiondata = kinect1.getSensorData();
        terrainAnalysisResult.TerrainRecordAll(visiondata.get().gridMap,currentPm.pm);
        cout<<"big map recorded."<<endl;

     }
});



static auto visionThread = std::thread([]()
{
    while(true)
    {
        int a;
        visionPipe.recvInNrt(a);

        auto visiondata = kinect1.getSensorData();
        terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap);

        if(terrainAnalysisResult.Terrain != FlatTerrain)
        {
            /*Adjust x y z theta*/
            double paramAdjust[4] = {0, 0, 0, 0};
            bool adjustFinished = false;
            visionAdjust(paramAdjust, &adjustFinished);

            if(adjustFinished == false)
            {
                /*let robot move*/
                if(paramAdjust[3] != 0)
                {
                    visionWalkParam.movetype = turn;
                    visionWalkParam.turndata = paramAdjust[3];
                    visionWalkParam.totalCount = 6000/2;
                    cout<<"terrain turn"<<endl;
                }
                else
                {
                    visionWalkParam.movetype = flatmove;
                    memcpy(visionWalkParam.movedata,paramAdjust,3*sizeof(double));
                    visionWalkParam.totalCount = 5000/2;
                    cout<<"terrain move"<<endl;
                }
            }
            else
            {
                cout<<"Find the Position!!!"<<endl;
                cout<<"Begin Climb Up!!!"<<endl;
                visionWalkParam.movetype = stepup;
                switch (terrainAnalysisResult.Terrain)
                {
                case StepUpTerrain:
                {
                    /*the robot move body*/
                    cout<<"Begin Climb Up!!!"<<endl;
                    visionWalkParam.movetype = stepup;
                }
                    break;
                case StepDownTerrain:
                {
                    /*the robot move body*/
                    cout<<"Begin Climb Down!!!"<<endl;
                    visionWalkParam.movetype = stepdown;
                }
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            cout<<"FLAT TERRAIN MOVE"<<endl;
            cout<<"MOVE FORWARD: "<<0.325<<endl;
            double move_data[3] = {0, 0, 0.325};

            visionWalkParam.movetype = flatmove;
            visionWalkParam.totalCount = 5000/2;
            memcpy(visionWalkParam.movedata,move_data,sizeof(move_data));
        }

        isTerrainAnalysisFinished = true;
        cout<<"terrrainended"<<endl;
    }
});

auto visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    static bool isFirstTime = true;

    if (isTerrainAnalysisFinished)
    {
        if(isFirstTime)
        {
            visionWalkParam.count = 0;
            isFirstTime = false;
        }

        auto &robot = static_cast<Robots::RobotBase &>(model);

        switch(visionWalkParam.movetype)
        {
        case turn:
        {

            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case flatmove:
        {
            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;

        case bodymove:
        {
            RobotBody(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
            visionWalkParam.count++;
            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case stepup:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isTerrainAnalysisFinished = false;
            return 0;
        }
            break;
        case stepdown:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isTerrainAnalysisFinished = false;
            return 0;
        }
            break;
        }
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            visionPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}

auto stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

auto mapShot(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);

    matrix44 currentPm;
    robot.GetPmb(*currentPm.pm);


    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[0][0],currentPm.pm[0][1],currentPm.pm[0][2],currentPm.pm[0][3]);
    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[1][0],currentPm.pm[1][1],currentPm.pm[1][2],currentPm.pm[1][3]);
    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[2][0],currentPm.pm[2][1],currentPm.pm[2][2],currentPm.pm[2][3]);
    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[3][0],currentPm.pm[3][1],currentPm.pm[3][2],currentPm.pm[3][3]);

    visionRecordPipe.sendToNrt(currentPm);
    return 0;
}


auto visionRecordParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

int main(int argc, char *argv[])
{   
    kinect1.start();

    PushDoorSimple::PushDoorSimpleWrapper::StartReceiveData();
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address ="/home/hex/Desktop/RobotIII/resource/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/Desktop/RobotIII/resource/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionStairs_Test/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    rs.addCmd("gus", GoStair::parseGoUpStair,GoStair::GoUpStair);
    rs.addCmd("gds", GoStair::parseGoDownStair,GoStair::GoDownStair);

    rs.addCmd("vwk", visionWalkParse, visionWalk);
    rs.addCmd("swk", stopVisionWalkParse, visionWalk);
    rs.addCmd("record",visionRecordParse,mapShot);

    rs.addCmd("climb",Rofo::rofoParse,Rofo::rofoGait);
    rs.addCmd("edcl", Rofo::rofoEndParse,Rofo::rofoEndGait);
    rs.addCmd("ay",Rofo::ayParse,Rofo::ayGait);

    //liujimu's gaits
    rs.addCmd("cwf", CWFParse, CWFGait);
    rs.addCmd("cwfs", CWFStopParse, CWFGait);
    rs.addCmd("pr", pushRecoveryParse, pushRecoveryGait);
    rs.addCmd("prs", pushRecoveryStopParse, pushRecoveryGait);
    rs.addCmd("cmb", parseContinuousMoveBegin, continuousMove);
    rs.addCmd("cmj", parseContinuousMoveJudge, continuousMove);
    rs.addCmd("tw", twistWaistParse, twistWaistGait);
    rs.addCmd("sh", sayHelloParse, sayHelloGait);
    rs.addCmd("ph", pegInHoleParse, pegInHoleGait);
    //zhaoyue
    rs.addCmd("sw", swingParse, swingGait);
    //sunqiao
    rs.addCmd("psd", PushDoorSimple::PushDoorSimpleWrapper::ParseCmds,
                     PushDoorSimple::PushDoorSimpleWrapper::GaitFunction);
    rs.addCmd("cpp", CrowdPassing::CrowdPassingGaitWrapper::ParseCmds,
                     CrowdPassing::CrowdPassingGaitWrapper::GaitFunction);


    Rofo::RofoWalkInit();
    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;

}
