#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

#include "Motions.h"
#include "rtdk.h"


#define GoUpStairLength_Enough 120000
static double Gait_GoUpStair[18*GoUpStairLength_Enough];
static int GaitStairCount;
 void GoStair::parseGoUpStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
    GoStairParam param;
    for (auto &i:params)
    {
        if(i.first=="file")
        {
     /*       static std::map<std::string, std::tuple<int, std::unique_ptr<double> > > walkFileMap;

            const auto found = walkFileMap.find(i.second);
            if (found != walkFileMap.end())
            {
                //步态已经存在
                std::tie(param.gaitCount, std::ignore) = found->second;
                param.pIn = std::get<3>(found->second).get();
                std::cout << "gostair count:" << param.count << std::endl;

            }
            else
            {
                if (walkFileMap.size() > 1)
                {
                    throw std::runtime_error("only one path is allowed");
                }

                //插入步态*/
                std::ifstream file;
                std::string FileName = i.second;
                cout<<"file name:"<<FileName<<endl;

               int gaitNum{ -1 };

                file.open(FileName);
                if (!file) throw std::logic_error("gostair file not exist");
                for (double tem; !file.eof(); file >> tem) ++gaitNum;
                if (gaitNum % 18 != 0) throw std::logic_error("gostair file invalid, because the num of numbers is not valid");
                gaitNum /= 18;
                file.close();

                param.gaitCount = gaitNum;
                GaitStairCount=gaitNum;

               // std::unique_ptr<double> p(new double[gaitNum* 18]);


               // Gait_GoUpStair=p.get();


                file.open(FileName);
                for (int i = 0; !file.eof(); file >> Gait_GoUpStair[i++]);
                file.close();

                cout<<"gaitnum:"<<gaitNum<<endl;
               // cout<<"pin 1: "<<param.pIn[0]<<" "<<param.pIn[17]<<" "<<param.pIn[18]<<endl;
               // cout<<"pin end: "<<param.pIn[gaitNum*18-17]<<" "<<param.pIn[gaitNum*18-16]<<" "<<param.pIn[gaitNum*18-1]<<endl;
               // cout<<"pin 1 global poiter: "<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[17]<<" "<<Gait_GoUpStair[18]<<endl;

              //  walkFileMap.insert(std::make_pair(i.second, std::make_tuple(gaitNum, std::move(p))));
            //}
        }

        else
        {
            throw std::logic_error("internal error happened, because invalid params in parseFastWalk");
        }
    }

    msg.copyStruct(param);

 }

 void GoStair::parseGoDownStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
     GoStairParam param;
     param.gaitCount=GaitStairCount;
     msg.copyStruct(param);
 }

int GoStair::GoUpStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<const GoStair::GoStairParam &>(param_in);
    if(pSP.count==0)
    {
        cout<<"gostair begins"<<"gait length:"<<pSP.gaitCount<<endl;
        cout<<"pin value"<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[1]<<" "<<Gait_GoUpStair[17]<<endl;
    }


    if(pSP.count < pSP.gaitCount)
    {
         robot.SetPin(&Gait_GoUpStair[pSP.count*18]);
    }

   return pSP.gaitCount-pSP.count-1;

}


int GoStair::GoDownStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

     auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<const GoStair::GoStairParam &>(param_in);
    if(pSP.count==0)
    {
        cout<<"go down stair begins"<<"gait length:"<<pSP.gaitCount<<endl;
        cout<<"pin value"<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[1]<<" "<<Gait_GoUpStair[17]<<endl;
    }


    if(pSP.count < pSP.gaitCount)
    {
         robot.SetPin(&Gait_GoUpStair[(pSP.gaitCount-pSP.count-1)*18]);
    }

   return pSP.gaitCount-pSP.count-1;

}


/*int fastWalk(ROBOT_BASE * pRobot, const GAIT_PARAM_BASE * pParam)
{
    auto pFP = static_cast<const FAST_WALK_PARAM*>(pParam);

    if (pFP->count < pFP->accCount)
    {
        pRobot->SetPin(&pFP->pInAcc[pFP->count * 18]);
    }
    else if ((pFP->count- pFP->accCount) < ((pFP->n - 1) * pFP->constCount))
    {
        int count = (pFP->count - pFP->accCount) % pFP->constCount;
        pRobot->SetPin(&pFP->pInConst[count * 18]);
    }
    else
    {
        int count = pFP->count - pFP->accCount - ((pFP->n - 1) * pFP->constCount);
        pRobot->SetPin(&pFP->pInDec[count * 18]);
    }

    return pFP->accCount + ((pFP->n - 1) * pFP->constCount) + pFP->decCount - pFP->count-1;
}

*/

/*
Aris::Core::MSG parseFastWalk(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    FAST_WALK_PARAM param;
    for (auto &i : params)
    {
        if (i.first == "file")
        {
            static std::map<std::string, std::tuple<int, int, int, std::unique_ptr<double> > > walkFileMap;

            const auto found = walkFileMap.find(i.second);
            if (found != walkFileMap.end())
            {
                 std::tie(param.accCount, param.constCount, param.decCount, std::ignore) = found->second;
                param.pInAcc = std::get<3>(found->second).get();
                param.pInConst = std::get<3>(found->second).get() + 18* param.accCount;
                param.pInDec = std::get<3>(found->second).get() + 18 * (param.accCount + param.constCount);


            }
            else
            {
                if (walkFileMap.size() > 1)
                {
                    throw std::runtime_error("only one path is allowed");
                }

                 std::ifstream file;
                std::string accFile = i.second + "_acc.txt";
                std::string constFile = i.second + "_const.txt";
                std::string decFile = i.second + "_dec.txt";

                int accNum{ -1 }, decNum{ -1 }, constNum{ -1 };

                file.open(accFile);
                if (!file) throw std::logic_error("acc file not exist");
                for (double tem; !file.eof(); file >> tem) ++accNum;
                if (accNum % 18 != 0) throw std::logic_error("acc file invalid, because the num of numbers is not valid");
                accNum /= 18;
                file.close();

                param.accCount = accNum;

                std::unique_ptr<double> p(new double[(accNum + constNum + decNum) * 18]);

                param.pInAcc = p.get();
                param.pInConst = p.get() + accNum * 18;
                param.pInDec = p.get() + (accNum + constNum) * 18;

                file.open(accFile);
                for (int i = 0; !file.eof(); file >> param.pInAcc[i++]);
                file.close();

                file.open(constFile);
                for (int i = 0; !file.eof(); file >> param.pInConst[i++]);
                file.close();

                file.open(decFile);
                for (int i = 0; !file.eof(); file >> param.pInDec[i++]);
                file.close();

                walkFileMap.insert(std::make_pair(i.second, std::make_tuple(accNum, constNum, decNum, std::move(p))));

            }
        }
        else if (i.first == "n")
        {
            param.n = std::stoi(i.second);
        }
        else
        {
            throw std::logic_error("internal error happened, because invalid params in parseFastWalk");
        }
    }

    Aris::Core::MSG msg;

    msg.CopyStruct(param);

    return msg;

}*/



