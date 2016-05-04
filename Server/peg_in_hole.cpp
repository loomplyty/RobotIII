#include "peg_in_hole.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto pegInHoleParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    phParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "holeDepth")
        {
            param.holeDepth = std::stod(i.second);
        }
        else if (i.first == "holeDepth")
        {
            param.holeDepth = std::stod(i.second);
        }
    }

    msg.copyStruct(param);
}

auto pegInHoleGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const phParam &>(param_in);

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    //阻抗控制参数
    double F[3]{ 0, 0, 0 };
    double M[3]{ 1, 1, 1 };
    double K[3]{ 0, 0, 0 };
    double C[3]{ 50, 50, 50 };
    double bodyAcc[3]{ 0 };
    static double bodyVel[3]{ 0 };
    static double bodyDisp[3]{ 0 };
    static double maxVel{ 0 };
    const double kClockPeriod{ 0.001 };

    //力传感器数据
    static double forceOffsetSum[6]{ 0 };
    double forceOffsetAvg[6]{ 0 };
    double realForceData[6]{ 0 };
    double forceInBody[6]{ 0 };
    const double kForceThreshold[6]{ 20, 20, 20, 30, 30, 30 }; //力传感器的触发阈值,单位N或Nm
    const double kForceMax[6]{ 100, 100, 100, 50, 50, 50 }; //力的上限

    //找孔中心
    static mbParam mb_param;
    static double contactPos[3][3]{ 0 };
    static double holeCenterPos[3]{ 0 };
    static PegInHoleProcess process{ PegInHoleProcess::PREPARE };
    static std::int32_t step{ 0 };
    static std::int32_t beginCount{ 0 };
    static bool returnFlag{ false };
    static bool alignFlag{ false };
    const double kClearance{ 0.04 };

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    //力传感器手动清零
    if (param.count < 100)
    {
        if(param.count == 0)
        {
            std::fill(forceOffsetSum, forceOffsetSum + 6, 0);

            std::fill(bodyDisp, bodyDisp + 3, 0);
            std::fill(bodyVel, bodyVel + 3, 0);
            maxVel = 0;
            std::fill(holeCenterPos, holeCenterPos + 3, 0);
            process = PegInHoleProcess::PREPARE;
            step = 0;
            beginCount = 0;
            returnFlag = false;
            alignFlag = false;
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
            realForceData[i] = param.force_data->at(0).fce[i] - forceOffsetAvg[i];
            //若力超过设定的最大值，则只取最大值
            if(std::fabs(realForceData[i]) > kForceMax[i])
            {
                realForceData[i] = realForceData[i] / std::fabs(realForceData[i]) * kForceMax[i];
            }
        }
        //转换到机器人身体坐标系
        aris::dynamic::s_f2f(*robot.forceSensorMak().prtPm(), realForceData, forceInBody);
        //显示力的初始值
        if(param.count == 100)
        {
            rt_printf("forceOffsetAvg: %f %f %f %f %f %f\n",
                      forceOffsetAvg[0], forceOffsetAvg[1], forceOffsetAvg[2],
                      forceOffsetAvg[3], forceOffsetAvg[4], forceOffsetAvg[5]);
        }

        //人手推动机器人身体移动
        if (process == PegInHoleProcess::PREPARE)
        {
            //以绕Y轴的力矩作为判断进入下一流程的标志
            if (std::fabs(forceInBody[4]) > kForceThreshold[4])
            {
                alignFlag = true;
            }
            double forceInBodyMag = std::sqrt(std::pow(forceInBody[0], 2) + std::pow(forceInBody[2], 2));
            if (alignFlag && forceInBodyMag < param.contactForce / 2 && maxVel < 0.0001)
            {
                process = PegInHoleProcess::ALIGN;
                std::fill(bodyVel, bodyVel + 3, 0);
                step = 0;

                //for test
                rt_printf("Preparation finished.\n\n");
                rt_printf("\nProcess: %d\n", (int)process);
                rt_printf("Step: %d\n", (int)step);
                rt_printf("ForceInBody: %f %f %f\n", forceInBody[0], forceInBody[1], forceInBody[2]);
                rt_printf("Peb: %f %f %f\n\n", Peb[0], Peb[1], Peb[2]);
            }

            double maxForce{ 0 };
            int index{ 0 };
            for (int i = 0; i < 3; i++)
            {
                if (std::fabs(forceInBody[i]) > maxForce)
                {
                    maxForce = std::fabs(forceInBody[i]);
                    index = i;
                }
            }
            if (maxForce > kForceThreshold[index])
            {
                for (int i = 0; i < 3; i++)
                {
                    F[i] = 2 * forceInBody[i] / kForceMax[i];
                }
            }

            maxVel = 0;
            for (int i = 0; i < 3; i++)
            {
                bodyAcc[i] = (F[i] - C[i] * bodyVel[i] - K[i] * bodyDisp[i]) / M[i];
                bodyVel[i] += bodyAcc[i] * kClockPeriod;
                bodyDisp[i] += bodyVel[i] * kClockPeriod;

                if (std::fabs(bodyVel[i]) > maxVel)
                {
                    maxVel = std::fabs(bodyVel[i]);
                }
            }

            std::copy(bodyDisp, bodyDisp + 3, Peb);
            robot.SetPeb(Peb, beginMak);
            robot.SetPee(Pee, beginMak);
        }

        //碰边，找孔中心
        else if (process == PegInHoleProcess::ALIGN)
        {
            //设置运动方向，三个接触点构成一个直角三角形
            if (step == 0)
            {
                F[0] = 1;
                F[1] = 0;
                F[2] = 0;
            }
            else if (step == 1)
            {
                F[0] = -std::sqrt(0.5);
                F[1] = 0;
                F[2] = -std::sqrt(0.5);
            }
            else if (step == 2)
            {
                F[0] = -std::sqrt(0.5);
                F[1] = 0;
                F[2] = std::sqrt(0.5);
            }
            //使用阻抗模型计算身体位移、速度、加速度
            for (int i = 0; i < 3; i++)
            {
                bodyAcc[i] = (F[i] / 2 - C[i] * bodyVel[i] - K[i] * bodyDisp[i]) / M[i];
                bodyVel[i] += bodyAcc[i] * kClockPeriod;
                bodyDisp[i] += bodyVel[i] * kClockPeriod;
            }
            std::copy(bodyDisp, bodyDisp + 3, Peb);
            robot.SetPeb(Peb, beginMak);
            robot.SetPee(Pee, beginMak);

            //当接触力超过阈值时，记录当前身体坐标为接触点坐标，并开始寻找下一个接触点
            double forceInBodyMag = std::sqrt(std::pow(forceInBody[0], 2) + std::pow(forceInBody[2], 2));
            if (param.count - beginCount > 500 && forceInBodyMag > param.contactForce)
            {
                std::copy(Peb, Peb + 3, contactPos[step]);
                std::fill(bodyVel, bodyVel + 3, 0);
                ++step;
                beginCount = param.count + 1;
                //for test
                rt_printf("\nProcess: %d\n", (int)process);
                rt_printf("Step: %d\n", (int)step);
                rt_printf("ForceInBody: %f %f %f\n", forceInBody[0], forceInBody[1], forceInBody[2]);
                rt_printf("Peb: %f %f %f\n\n", Peb[0], Peb[1], Peb[2]);

                //完成三次触碰后，取首尾两个接触点中点为孔中心，并进入下一流程
                if (step > 2)
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        holeCenterPos[i] = (contactPos[0][i] + contactPos[2][i]) / 2;
                    }
                    process = PegInHoleProcess::INSERT;
                    step = 0;
                    //for test
                    rt_printf("Alignment finished.\n\n");
                    rt_printf("beginCount: %d\n", beginCount);
                    rt_printf("holeCenterPos: %f %f %f\n", holeCenterPos[0], holeCenterPos[1], holeCenterPos[2]);
                }
            }
        }

        //插入
        else if (process == PegInHoleProcess::INSERT)
        {
            mb_param.count = param.count - beginCount;
            mb_param.totalCount = param.totalCount;
            mb_param.beginMak = &beginMak;
            std::copy(holeCenterPos, holeCenterPos + 3, mb_param.targetPeb);
            //Step 0: 水平移动到孔中心
            //Step 1: 垂直向上插入
            if (step == 1)
            {
                mb_param.targetPeb[1] += param.holeDepth;
                mb_param.totalCount = 2 * mb_param.totalCount;
            }
            int ret = moveBodyGait(robot, mb_param);
            if (ret == 0)
            {
                ++step;
                beginCount = param.count + 1;
                //for test
                rt_printf("\nProcess: %d\n", (int)process);
                rt_printf("Step: %d\n", (int)step);
                rt_printf("ForceInBody: %f %f %f\n", forceInBody[0], forceInBody[1], forceInBody[2]);
                rt_printf("Peb: %f %f %f\n\n", Peb[0], Peb[1], Peb[2]);

                if (step > 1)
                {
                    process = PegInHoleProcess::RETURN;
                    step = 0;
                    //for test
                    rt_printf("Insertion finished.\n\n");
                }
            }
        }

        //抽出并返回原位
        else if (process == PegInHoleProcess::RETURN)
        {
            if (!returnFlag)
            {
                if( forceInBody[1] < -kForceThreshold[1] )
                {
                    beginCount = param.count + 1;
                    returnFlag = true;
                }
            }
            else
            {
                mb_param.count = param.count - beginCount;
                mb_param.totalCount = param.totalCount;
                mb_param.beginMak = &beginMak;
                //Step 0: 垂直向下抽出
                //Step 1: 身体返回原位
                if (step == 0)
                {
                    std::copy(holeCenterPos, holeCenterPos + 3, mb_param.targetPeb);
                    mb_param.targetPeb[1] -= kClearance;
                    mb_param.totalCount = 2 * mb_param.totalCount;
                }
                else if (step == 1)
                {
                    std::fill(mb_param.targetPeb, mb_param.targetPeb + 6, 0);
                }
                int ret = moveBodyGait(robot, mb_param);
                if (ret == 0)
                {
                    ++step;
                    beginCount = param.count + 1;
                    //for test
                    rt_printf("\nProcess: %d\n", (int)process);
                    rt_printf("Step: %d\n", (int)step);
                    rt_printf("ForceInBody: %f %f %f\n", forceInBody[0], forceInBody[1], forceInBody[2]);
                    rt_printf("Peb: %f %f %f\n\n", Peb[0], Peb[1], Peb[2]);

                    if (step > 1)
                    {
                        process = PegInHoleProcess::STOP;
                        //for test
                        rt_printf("Body returned.\n\n");
                    }
                }
            }
        }

        //for test
//        if (param.count % 2000 == 0)
//        {
//            rt_printf("\nForceInBody: %f %f %f\n", forceInBody[0], forceInBody[1], forceInBody[2]);
//            rt_printf("Peb: %f %f %f\n\n", Peb[0], Peb[1], Peb[2]);
//        }
    }

    if (process == PegInHoleProcess::STOP)
    {
        //for test
        rt_printf("Gait finished.\n");

        return 0;
    }
    else
    {
        return 1;
    }
}

auto moveBodyGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const mbParam &>(param_in);

    //初始化
    static double beginPee[18], beginPeb[6];
    static double targetPeb[6];
    if (param.count == 0)
    {
        robot.GetPee(beginPee, *param.beginMak);
        robot.GetPeb(beginPeb, *param.beginMak);
        robot.GetPeb(targetPeb, *param.beginMak);
        std::copy(param.targetPeb, param.targetPeb + 3, targetPeb);
    }

    double Peb[6], Pee[18];
    std::copy(beginPee, beginPee + 18, Pee);
    const double s = -0.5 * std::cos(PI * (param.count  + 1) / param.totalCount ) + 0.5; //s从0到1
    for (int i = 0; i < 6; ++i)
    {
        Peb[i] = beginPeb[i] * (1 - s) + targetPeb[i] * s;
    }
    robot.SetPeb(Peb, *param.beginMak);
    robot.SetPee(Pee, *param.beginMak);

    //for test
//    if (param.count % 1000 == 0)
//    {
//        rt_printf("mb_count: %d\n", param.count);
//        rt_printf("mb_s: %f\n", s);
//        rt_printf("mb_beginPeb:\t %f %f %f\n", beginPeb[0], beginPeb[1], beginPeb[2]);
//        rt_printf("mb_targetPeb:\t %f %f %f\n", targetPeb[0], targetPeb[1], targetPeb[2]);
//        rt_printf("mb_Peb:\t\t %f %f %f\n", Peb[0], Peb[1], Peb[2]);
//    }

    return param.totalCount - param.count - 1;
}
