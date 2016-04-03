#include "push_recovery.h"

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
      <pr default="pr_param">
        <pr_param type="group">
          <pushCount abbreviation="p" type="int" default="1000"/>
          <recoverCount abbreviation="r" type="int" default="4000"/>
          <totalCount abbreviation="t" type="int" default="5000"/>
          <firstStepCount abbreviation="f" type="int" default="2000"/>
          <distance abbreviation="d" type="double" default="0.3"/>
          <height abbreviation="h" type="double" default="0.05"/>
          <angle abbreviation="a" type="double" default="5"/>
          <descend abbreviation="c" type="double" default="0.025"/>
        </pr_param>
      </pr>
      <prs/>
*/

auto pushRecoveryGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const prParam &>(param_in);

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double bodyPe[6];
    static double bodyVel[6];
    static double recoverBodyPe[6];//插值起点
    static double recoverBodyVel[6];//插值起点

    static bool isWalking{ false };
    static int beginCount{ 0 };
    const double clockPeriod{ 0.001 };

    static int fAxis;
    static int fSign;
    static double forceOffsetSum[6]{ 0 };
    double forceOffsetAvg[6]{ 0 };
    double realForceData[6]{ 0 };
    double forceInBody[6]{ 0 };
    const double forceThreshold[6]{ 40, 40, 40, 100, 100, 100 };//力传感器的触发阈值,单位N或Nm
    const double forceAMFactor{ 1 };//力传感器输出数值与实际作用力的比值，1或1000
    //const double sensorPe[6]{ 0, 0, 0, PI, PI/2, 0 };

    double M{ 1 };
    double C[6]{ 1.5, 1.5, 1.5, 1.5, 1.5, 1.5 };
    double K[6]{ 0, 1.14, 0, 1.14, 0, 0 };
    double Fh = 1.5 * param.d;
    double Fv = 2.5 * param.descend;
    double Fr = 0.0427 * param.angle;

    //力传感器手动清零
    if (param.count < 100)
    {
        if(param.count == 0)
        {
            std::fill(forceOffsetSum, forceOffsetSum + 6, 0);
        }
        for(int i = 0; i < 6; i++)
        {
            forceOffsetSum[i] += param.force_data->at(0).fce[i];
        }
    }
    else
    {
        for(int i = 0; i < 6; i++)
        {
            forceOffsetAvg[i] = forceOffsetSum[i] / 100;
            realForceData[i]=(param.force_data->at(0).fce[i] - forceOffsetAvg[i]) / forceAMFactor;
            //转换到机器人身体坐标系
//            double sensorPm[16]{ 0 };
//            aris::dynamic::s_pe2pm(sensorPe, sensorPm);
//            aris::dynamic::s_pm_dot_v3(sensorPm, realForceData, forceInBody);
//            aris::dynamic::s_pm_dot_v3(sensorPm, realForceData + 3, forceInBody + 3);
            aris::dynamic::s_f2f(*robot.forceSensorMak().prtPm(), realForceData, forceInBody);
        }

        //for test
        if(param.count % 1000 == 0)
        {
            rt_printf("forceInBody: %f %f %f %f %f %f\n",forceInBody[0],forceInBody[1],forceInBody[2],forceInBody[3],forceInBody[4],forceInBody[5]);
        }

        /*检测力的方向，确定运动参数*/
        if(!isWalking)
        {
            int maxIndex{ -1 };
            double maxForce{ 0 };
            for(int i = 0; i < 3; i++)
            {
                double tmpForce = std::fabs(forceInBody[i]) - forceThreshold[i];
                if(tmpForce > maxForce)
                {
                    maxIndex = i;
                    maxForce = tmpForce;
                }
            }

            if(maxIndex >= 0)
            {
                fAxis = maxIndex;
                fSign = forceInBody[fAxis] / std::fabs(forceInBody[fAxis]);
                //显示力的方向
                rt_printf("PushDirection: %c%c\n", 44 - fSign, fAxis + 'x');

                isWalking=true;
                //初始化
                beginCount = param.count + 1;
                beginMak.setPrtPm(*robot.body().pm());
                beginMak.update();
                robot.GetPee(beginPee, beginMak);
                std::fill(bodyPe, bodyPe + 6, 0);
                std::fill(bodyVel, bodyVel + 6, 0);
            }
        }

        /*运动规划*/
        else
        {            
            double Peb[6], Pee[18];
            std::fill(Peb, Peb + 6, 0);
            std::copy(beginPee, beginPee + 18, Pee);

            double d = fSign * param.d;
            double h = param.h;
            int count = param.count - beginCount;
            int totalCount = param.totalCount;
            double F[6]{0};


            /*设置身体*/
            if(count < param.pushCount)
            {
                if(fAxis == 1)
                {
                    F[fAxis] = 2.5 * fSign * Fv;
                }
                else
                {
                    F[fAxis] = fSign * Fh;
                    F[1] = -1 * Fv;
                    F[3] = (1 - fAxis) * fSign * Fr;
                }
            }
            //用阻抗模型计算身体位姿变化
            double bodyAcc[6]{ 0 };
            if(count < param.recoverCount)
            {
                for (int i = 0; i < 4; i++)
                {
                    bodyAcc[i] = (F[i] - C[i] * bodyVel[i] - K[i] * bodyPe[i]) / M;
                    bodyVel[i] += bodyAcc[i] * clockPeriod;
                    bodyPe[i] += bodyVel[i] * clockPeriod;
                    std::copy(bodyPe, bodyPe + 6, Peb);
                }
                if(count == (param.recoverCount - 1))
                {
                    std::copy_n(bodyPe, 6, recoverBodyPe);
                    std::copy_n(bodyVel, 6, recoverBodyVel);
                }
            }
            //超过recoverCount后，用两点Hermite插值调整身体至目标位姿
            else if(count < totalCount)
            {
                double finalBodyPE[6]{0};
                if(fAxis != 1)
                {
                    finalBodyPE[fAxis] = d;
                }
                for (int i = 0; i < 4; i++)
                {
                    Peb[i] = Hermite3(count * clockPeriod, (param.recoverCount - 1) * clockPeriod, (param.totalCount - 1) * clockPeriod,
                                         recoverBodyPe[i], finalBodyPE[i], recoverBodyVel[i], 0);
                }
            }

            //规划腿
            if(fAxis != 1)
            {
                if(count < param.firstStepCount)
                {
                    double s = -(PI / 2) * cos(PI * (count + 1) / param.firstStepCount) + PI / 2;
                    /*设置移动腿*/
                    for (int i = 0; i < 18; i += 6)
                    {
                        Pee[i + 1] = h*sin(s) + beginPee[i + 1];
                        Pee[i + fAxis] = d / 2 * (1 - cos(s)) + beginPee[i + fAxis];
                    }
                    /*设置支撑腿*/
                    for (int i = 3; i < 18; i += 6)
                    {
                        Pee[i + 1] = beginPee[i + 1];
                        Pee[i + fAxis] = beginPee[i + fAxis];
                    }
                }
                else if(count < totalCount)
                {
                    double s = -(PI / 2) * cos(PI * (count + 1 - param.firstStepCount) / (totalCount - param.firstStepCount)) + PI / 2;
                    /*设置移动腿*/
                    for (int i = 3; i < 18; i += 6)
                    {
                        Pee[i + 1] = h*sin(s) + beginPee[i + 1];
                        Pee[i + fAxis] = d / 2 * (1 - cos(s)) + beginPee[i + fAxis];
                    }
                    /*设置支撑腿*/
                    for (int i = 0; i < 18; i += 6)
                    {
                        Pee[i + 1] = beginPee[i + 1];
                        Pee[i + fAxis] = beginPee[i + fAxis] + d;
                    }
                }
            }

            char order[4] = "313";
            if(fAxis == 2)
            {
                std::strcpy(order, "123");
            }
            robot.SetPeb(Peb, beginMak, order);
            robot.SetPee(Pee, beginMak);

            //判断动作结束
            if(count == (totalCount - 1))
            {
                isWalking = false;
                fSign = 0;
            }
        }
    }

    if(PrState::getState().isStopping() && (!isWalking))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

auto pushRecoveryParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    prParam param;

    for (auto &i : params)
    {
        if (i.first == "pushCount")
        {
            param.pushCount = std::stoi(i.second);
        }
        if (i.first == "recoverCount")
        {
            param.recoverCount = std::stoi(i.second);
        }
        else if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "firstStepCount")
        {
            param.firstStepCount = std::stoi(i.second);
        }
        else if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = stod(i.second);
        }
        else if (i.first == "angle")
        {
            param.angle = stod(i.second);
        }
        else if (i.first == "descend")
        {
            param.descend = stod(i.second);
        }
    }

    PrState::getState().isStopping() = false;

    msg.copyStruct(param);
}

auto pushRecoveryStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    PrState::getState().isStopping() = true;
}

