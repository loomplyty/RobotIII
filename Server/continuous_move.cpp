#include "continuous_move.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

/*将以下注释代码添加到xml文件*/
/*
      <cmb default="cmb_param">
        <cmb_param type="group">
          <u abbreviation="u" type="double" default="0"/>
          <v abbreviation="v" type="double" default="0"/>
          <w abbreviation="w" type="double" default="0"/>
          <roll abbreviation="r" type="double" default="0"/>
          <pitch abbreviation="p" type="double" default="0"/>
          <yaw abbreviation="y" type="double" default="0"/>
        </cmb_param>
      </cmb>
      <cmj default="cmj_param">
        <cmj_param type="group">
          <isStop abbreviation="s" type="int" default="0"/>
          <isForce abbreviation="f" type="int" default="0"/>
          <u abbreviation="u" type="int" default="0"/>
          <v abbreviation="v" type="int" default="0"/>
          <w abbreviation="w" type="int" default="0"/>
          <roll abbreviation="r" type="int" default="0"/>
          <pitch abbreviation="p" type="int" default="0"/>
          <yaw abbreviation="y" type="int" default="0"/>
        </cmj_param>
      </cmj>
*/

std::atomic_bool isForce;
std::atomic_bool isContinue;
std::atomic_int moveDir[6];

void parseContinuousMoveBegin(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    ContinuousMoveParam param;

    for(auto &i:params)
    {
        if(i.first=="u")
        {
            moveDir[0]=std::stoi(i.second);
        }
        else if(i.first=="v")
        {
            moveDir[1]=std::stoi(i.second);
        }
        else if(i.first=="w")
        {
            moveDir[2]=std::stoi(i.second);
        }
        else if(i.first=="yaw")
        {
            moveDir[3]=std::stoi(i.second);
        }
        else if(i.first=="pitch")
        {
            moveDir[4]=std::stoi(i.second);
        }
        else if(i.first=="roll")
        {
            moveDir[5]=std::stoi(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    isContinue=true;
    isForce=true;

    msg.copyStruct(param);

    std::cout<<"finished parse"<<std::endl;
}

void parseContinuousMoveJudge(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for(auto &i:params)
    {
        if(i.first=="isStop")
        {
            if(i.second=="0")
                isContinue=true;
            else
                isContinue=false;
        }
        else if(i.first=="isForce")
        {
            if(i.second=="1")
                isForce=true;
            else
                isForce=false;
        }
        else if(i.first=="u")
        {
            moveDir[0]=stoi(i.second);
        }
        else if(i.first=="v")
        {
            moveDir[1]=stoi(i.second);
        }
        else if(i.first=="w")
        {
            moveDir[2]=stoi(i.second);
        }
        else if(i.first=="yaw")
        {
            moveDir[3]=stoi(i.second);
        }
        else if(i.first=="pitch")
        {
            moveDir[4]=stoi(i.second);
        }
        else if(i.first=="roll")
        {
            moveDir[5]=stoi(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    std::cout<<"finished parse"<<std::endl;
}

/*****C & forceRatio must be adjusted when used on different robots*****/
int continuousMove(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const ContinuousMoveParam &>(param_in);

    double bodyVel[6];
    double bodyAcc[6];
    double bodyPm[4][4];
    double deltaPE[6];
    double deltaPm[4][4];
    double realPE[6];
    double realPm[4][4];
    double nowPee[18];

    double Fbody[6]{0,0,0,0,0,0};
    double C[6]{30,30,30,30,30,30};
    double M[6]{1,1,1,1,1,1};
    double deltaT{0.001};
    double forceRange[6]{30,30,30,20,20,20};
    double forceRatio{1};//1 on RobotIII, 1000 on RobotVIII & single motor
    double forceInBody[6];

    static CM_RecordParam CMRP;

    if(isContinue==true)
    {
        //rt_printf("gait continuing\n");
        if(isForce==false)
        {
            for (int i=0;i<6;i++)
            {
                Fbody[i]=moveDir[i];
            }
        }
        else
        {
            if(param.count<100)
            {
                //initialize
                if (param.count==0)
                {
                    for(int i=0;i<6;i++)
                    {
                        CMRP.forceSum[i]=0;
                        CMRP.bodyVel_last[i]=0;
                    }
                }

                CMRP.forceSum[0]+=param.force_data->at(0).Fx;
                CMRP.forceSum[1]+=param.force_data->at(0).Fy;
                CMRP.forceSum[2]+=param.force_data->at(0).Fz;
                CMRP.forceSum[3]+=param.force_data->at(0).Mx;
                CMRP.forceSum[4]+=param.force_data->at(0).My;
                CMRP.forceSum[5]+=param.force_data->at(0).Mz;
            }
            else if(param.count==100)
            {
                for(int i=0;i<6;i++)
                {
                    CMRP.forceAvg[i]=CMRP.forceSum[i]/100;
                }
            }
            else
            {
                CMRP.force[0]=(param.force_data->at(0).Fx-CMRP.forceAvg[0])/forceRatio;
                CMRP.force[1]=(param.force_data->at(0).Fy-CMRP.forceAvg[1])/forceRatio;
                CMRP.force[2]=(param.force_data->at(0).Fz-CMRP.forceAvg[2])/forceRatio;
                CMRP.force[3]=(param.force_data->at(0).Mx-CMRP.forceAvg[3])/forceRatio;
                CMRP.force[4]=(param.force_data->at(0).My-CMRP.forceAvg[4])/forceRatio;
                CMRP.force[5]=(param.force_data->at(0).Mz-CMRP.forceAvg[5])/forceRatio;

                aris::dynamic::s_f2f(*robot.forceSensorMak().prtPm(), CMRP.force, forceInBody);
                int num1;
                int num2;
                double fmax{0};
                double mmax{0};
                for (int i=0;i<3;i++)
                {
                    if (fabs(forceInBody[i])>fabs(fmax))
                    {
                        fmax=forceInBody[i];
                        num1=i;
                    }
                    if (fabs(forceInBody[i+3])>fabs(mmax))
                    {
                        mmax=forceInBody[i+3];
                        num2=i+3;
                    }
                }

                if(forceInBody[num2]>forceRange[num2])
                {
                    Fbody[num2]=1;
                }
                else if(forceInBody[num2]<-forceRange[num2])
                {
                    Fbody[num2]=-1;
                }
                else
                {
                    if(forceInBody[num1]>forceRange[num1])
                    {
                        Fbody[num1]=1;
                    }
                    else if(forceInBody[num1]<-forceRange[num1])
                    {
                        Fbody[num1]=-1;
                    }
                }
            }
        }

        for (int i=0;i<6;i++)
        {
            bodyAcc[i]=(Fbody[i]-C[i]*CMRP.bodyVel_last[i])/M[i];
            bodyVel[i]=CMRP.bodyVel_last[i]+bodyAcc[i]*deltaT;
            deltaPE[i]=bodyVel[i]*deltaT;
        }

        robot.GetPmb(*bodyPm);
        robot.GetPee(nowPee);
        double pBody[6];
        if(isForce==false)
        {
            aris::dynamic::s_pe2pm(deltaPE,*deltaPm,"213");
        }
        else
        {
            aris::dynamic::s_pe2pm(deltaPE,*deltaPm,"123");
        }
        aris::dynamic::s_pm_dot_pm(*bodyPm,*deltaPm,*realPm);
        aris::dynamic::s_pm2pe(*realPm,realPE,"313");


        robot.SetPeb(realPE);
        robot.SetPee(nowPee);

        if(param.count%1000==0)
        {
            rt_printf("count:%d\n",param.count);
            //rt_printf("bodyPm:%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",bodyPm[0][0],bodyPm[0][1],bodyPm[0][2],bodyPm[0][3],bodyPm[1][0],bodyPm[1][1],bodyPm[1][2],bodyPm[1][3],bodyPm[2][0],bodyPm[2][1],bodyPm[2][2],bodyPm[2][3],bodyPm[3][0],bodyPm[3][1],bodyPm[3][2],bodyPm[3][3]);
            rt_printf("RawForce:%f,%f,%f\n",param.force_data->at(0).Fx,param.force_data->at(0).Fy,param.force_data->at(0).Fz);
            rt_printf("ForceSum:%f,%f,%f\n",CMRP.forceSum[0],CMRP.forceSum[1],CMRP.forceSum[2]);
            rt_printf("ForceAvg:%f,%f,%f\n",CMRP.forceAvg[0],CMRP.forceAvg[1],CMRP.forceAvg[2]);
            rt_printf("Force:%f,%f,%f,%f,%f,%f\n",CMRP.force[0],CMRP.force[1],CMRP.force[2],CMRP.force[3],CMRP.force[4],CMRP.force[5]);
            rt_printf("realPE:%f,%f,%f,%f,%f,%f\n\n",realPE[0],realPE[1],realPE[2],realPE[3],realPE[4],realPE[5]);
        }

        memcpy(CMRP.bodyVel_last,bodyVel,sizeof(double)*6);
        return 1;
    }

    else
    {
        //rt_printf("gait stopping\n");
        for (int i=0;i<6;i++)
        {
            bodyAcc[i]=(Fbody[i]-C[i]*CMRP.bodyVel_last[i])/M[i];
            bodyVel[i]=CMRP.bodyVel_last[i]+bodyAcc[i]*deltaT;
            deltaPE[i]=bodyVel[i]*deltaT;
        }

        robot.GetPmb(*bodyPm);
        robot.GetPee(nowPee);
        aris::dynamic::s_pe2pm(deltaPE,*deltaPm,"213");
        aris::dynamic::s_pm_dot_pm(*bodyPm,*deltaPm,*realPm);
        aris::dynamic::s_pm2pe(*realPm,realPE,"313");

        robot.SetPeb(realPE);
        robot.SetPee(nowPee);

        memcpy(CMRP.bodyVel_last,bodyVel,sizeof(double)*6);

        if ( fabs(bodyVel[0])<1e-10 && fabs(bodyVel[1])<1e-10 && fabs(bodyVel[2])<1e-10 && fabs(bodyVel[3])<1e-10 && fabs(bodyVel[4])<1e-10 && fabs(bodyVel[5])<1e-10)
            return 0;
        else
            return 1;
    }
}

