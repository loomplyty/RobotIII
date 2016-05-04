#include "rofo2.h"
#include<iostream>
using namespace std;
using namespace  Rofo;

force_gait::GaitRobot rofo;

bool IsRofoEnd=false;

LogData temp_log_data;
// static variables for adjustY function
Ay_Param* pAP;

double adjustYTargetPee[18];





/*

//Aris::Core::MSG parseAdjust2Climb(const std::string &cmd, const map<std::string, std::string> &params)
//{
//    // origin
////    double firstEE[18] =
////    {
////        -0.3,-0.75,-0.65,
////        -0.45,-0.75,0,
////        -0.3,-0.75,0.65,
////        0.3,-0.75,-0.65,
////        0.45,-0.75,0,
////        0.3,-0.75,0.65,
////    };

////    double beginEE[18]
////    {
////        -0.3,-0.85,-0.65,
////        -0.45,-0.85,0,
////        -0.3,-0.85,0.65,
////        0.3,-0.85,-0.65,
////        0.45,-0.85,0,
////        0.3,-0.85,0.65,
////    };

////    obstacle
//    double firstEE[18] =
//    {
//        -0.35,-0.80,-0.50,
//        -0.45,-0.80,0,
//        -0.35,-0.80,0.50,
//        0.35,-0.80,-0.50,
//        0.45,-0.80,0,
//        0.35,-0.80,0.50,
//    };

//    double beginEE[18]
//    {
//        -0.35,-1.08,-0.50,
//        -0.45,-1.08,0,
//        -0.35,-1.08,0.50,
//        0.35,-1.08,-0.50,
//        0.45,-1.08,0,
//        0.35,-1.08,0.50,
//    };

//    Robots::ADJUST_PARAM  param;

//    std::copy_n(firstEE, 18, param.targetPee[0]);
//    std::fill_n(param.targetBodyPE[0], 6, 0);
//    std::copy_n(beginEE, 18, param.targetPee[1]);
//    std::fill_n(param.targetBodyPE[1], 6, 0);

//    param.periodNum = 2;
//    param.periodCount[0]=6000;
//    param.periodCount[1]=6000;

//    std::strcpy(param.relativeCoordinate,"B");
//    std::strcpy(param.relativeBodyCoordinate,"B");

//    for(auto &i:params)
//    {
//        if(i.first=="all")
//        {

//        }
//        else if(i.first=="first")
//        {
//            param.legNum=3;
//            param.motorNum=9;

//            param.legID[0]=0;
//            param.legID[1]=2;
//            param.legID[2]=4;

//            int motors[9] = { 0,1,2,6,7,8,12,13,14 };
//            std::copy_n(motors, 9, param.motorID);
//        }
//        else if(i.first=="second")
//        {
//            param.legNum=3;
//            param.motorNum=9;

//            param.legID[0]=1;
//            param.legID[1]=3;
//            param.legID[2]=5;

//            int motors[9] = { 3,4,5,9,10,11,15,16,17 };
//            std::copy_n(motors, 9, param.motorID);
//        }
//        else
//        {
//            std::cout<<"parse failed"<<std::endl;
//            return MSG{};
//        }
//    }

//    Aris::Core::MSG msg;

//    msg.CopyStruct(param);

//    std::cout<<"finished parse"<<std::endl;

//    return msg;
//}
*/

/*
//Aris::Core::MSG parseAdjustY(const std::string &cmd, const map<std::string, std::string> &params)
//{
//    // origin
////	double firstEE[18] =
////	{
////		-0.3,-0.75,-0.65,
////		-0.45,-0.75,0,
////		-0.3,-0.75,0.65,
////		0.3,-0.75,-0.65,
////		0.45,-0.75,0,
////		0.3,-0.75,0.65,
////	};

////	double beginEE[18]
////	{
////        -0.3,-0.85,-0.65,
////        -0.45,-0.85,0,
////        -0.3,-0.85,0.65,
////        0.3,-0.85,-0.65,
////        0.45,-0.85,0,
////        0.3,-0.85,0.65,
////	};

//    double ay=-0.85;

//    double firstEE[18] =
//    {
//        -0.35,-0.85,-0.50,
//        -0.45,-0.85,0,
//        -0.35,-0.85,0.50,
//        0.35,-0.85,-0.50,
//        0.45,-0.85,0,
//        0.35,-0.85,0.50,
//    };

//    double beginEE[18]
//    {
//        -0.35,-1.02,-0.50,
//        -0.45,-1.02,0,
//        -0.35,-1.02,0.50,
//        0.35,-1.02,-0.50,
//        0.45,-1.02,0,
//        0.35,-1.02,0.50,
//    };

////    Robots::ADJUST_PARAM  param;
//    ADY_PARAM param;

//    pAP=new ADY_PARAM;

//    for(auto &i:params)
//    {
//        if(i.first=="all")
//        {

//        }
//        else if(i.first=="first")
//        {
//            param.legNum=3;
//            param.motorNum=9;

//            param.legID[0]=0;
//            param.legID[1]=2;
//            param.legID[2]=4;

//            int motors[9] = { 0,1,2,6,7,8,12,13,14 };
//            std::copy_n(motors, 9, param.motorID);
//        }
//        else if(i.first=="second")
//        {
//            param.legNum=3;
//            param.motorNum=9;

//            param.legID[0]=1;
//            param.legID[1]=3;
//            param.legID[2]=5;

//            int motors[9] = { 3,4,5,9,10,11,15,16,17 };
//            std::copy_n(motors, 9, param.motorID);
//        }
//        else if(i.first=="ay")
//        {
//            ay=stod(i.second);
//            if(ay>-0.72||ay<-1.08)
//            {
//                ay=-0.85;
//                std::cout<<"ay out of range default set to %d"<<ay<<std::endl;
//            }
//            param.aY=ay;
//            param.IsRel=false;
//        }
//        else if(i.first=="dy")
//        {
//            param.dY=stod(i.second);
//            if(param.dY<-0.3||param.dY>0.3)
//            {
//                param.dY=-0.05;
//            }
//            param.IsRel=true;
//        }
//        else
//        {
//            std::cout<<"parse failed"<<std::endl;
//            return MSG{};
//        }
//    }
//    for(int i=0;i<6;i++)
//    {
//        firstEE[i*3+1]=ay;
//    }


//    std::copy_n(firstEE, 18, param.targetPee[0]);
//    std::fill_n(param.targetBodyPE[0], 6, 0);
//    std::copy_n(beginEE, 18, param.targetPee[1]);
//    std::fill_n(param.targetBodyPE[1], 6, 0);

//    param.periodNum = 1;
//    param.periodCount[0]=6000;
//    param.periodCount[1]=6000;

//    std::strcpy(param.relativeCoordinate,"B");
//    std::strcpy(param.relativeBodyCoordinate,"B");

//    Aris::Core::MSG msg;

//    msg.CopyStruct(param);

//    std::cout<<"finished parse ay"<<param.aY<<" "<<param.dY<<std::endl;

//    return msg;

//}
*/

auto Rofo::rofoEndParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    CLIMB_PARAM  param;
/*
    for(auto &i:params)
    {
        if(i.first=="count")
        {
            param.count=std::stoi(i.second);
        }
    }
*/
    msg.copyStruct(param);
    if(IsRofoEnd)
    {
        IsRofoEnd=false;
        std::cout<<"climb enabled"<<std::endl;

    }
    else
    {
        IsRofoEnd=true;
        std::cout<<"climb disabled"<<std::endl;
    }

    IsRofoEnd=true;

    std::cout<<"finished parse  endclimb"<<std::endl;
}

auto Rofo::rofoEndGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    rt_printf("end climb rt executed\n");
    auto &param = static_cast<const Robots::WalkParam &>(param_in);
    rt_printf("LAST TARGE; LAST FEEDBACK; TARGET; FEEDBACK;\n");
    for(int i=0;i<18;i++)
    {
        rt_printf("%d\t%d\t%d\t%d\n",param.last_motion_raw_data->at(i).target_pos
                  ,param.last_motion_raw_data->at(i).feedback_pos
                  ,param.motion_raw_data->at(i).target_pos
                  ,param.motion_raw_data->at(i).feedback_pos);
    }

    return 0;
}


int Rofo::RofoWalkInit()
{
//    rofo.Robot.loadXml(_XML_PATH);
    rofo.init();
//    robot_log.init();


    // need calcluate origin homed position then calculate the


    rofo.gait_part_leg[force_gait::LF].gait_ref_position_body_coord[0]=-0.318791579531186;
    rofo.gait_part_leg[force_gait::LF].gait_ref_position_body_coord[1]=-0.719675656557493;
    rofo.gait_part_leg[force_gait::LF].gait_ref_position_body_coord[2]=-0.500049789146799;

    rofo.gait_part_leg[force_gait::LM].gait_ref_position_body_coord[0]=-0.413084678293599;
    rofo.gait_part_leg[force_gait::LM].gait_ref_position_body_coord[1]=-0.719675656557493;
    rofo.gait_part_leg[force_gait::LM].gait_ref_position_body_coord[2]=0.0;

    rofo.gait_part_leg[force_gait::LR].gait_ref_position_body_coord[0]=-0.318791579531187;
    rofo.gait_part_leg[force_gait::LR].gait_ref_position_body_coord[1]=-0.719675656557493;
    rofo.gait_part_leg[force_gait::LR].gait_ref_position_body_coord[2]=0.498125689146798;

    rofo.gait_part_leg[force_gait::RF].gait_ref_position_body_coord[0]=0.318791579531186;
    rofo.gait_part_leg[force_gait::RF].gait_ref_position_body_coord[1]=-0.719675656557493;
    rofo.gait_part_leg[force_gait::RF].gait_ref_position_body_coord[2]=-0.500049789146799;

    rofo.gait_part_leg[force_gait::RM].gait_ref_position_body_coord[0]=0.413084678293599;
    rofo.gait_part_leg[force_gait::RM].gait_ref_position_body_coord[1]=-0.719675656557493;
    rofo.gait_part_leg[force_gait::RM].gait_ref_position_body_coord[2]=0.0;

    rofo.gait_part_leg[force_gait::RR].gait_ref_position_body_coord[0]=0.318791579531187;
    rofo.gait_part_leg[force_gait::RR].gait_ref_position_body_coord[1]=-0.719675656557493;
    rofo.gait_part_leg[force_gait::RR].gait_ref_position_body_coord[2]=0.498125689146798;

    // this is adjust the workspace
    for(int i=0;i<6;i++)
    {
        rofo.gait_part_leg[i].position_limit[1][0]=rofo.gait_part_leg[i].gait_ref_position_body_coord[1]-0.01;
        rofo.gait_part_leg[i].gait_ref_position_body_coord[1]-=0.30;

        // adjust contact force
        rofo.gait_part_leg[i].limit_y_positive_hi=800;


    }
    rofo.gait_part_leg[force_gait::LR].limit_y_positive_hi=900.0;
    rofo.gait_part_leg[force_gait::LM].limit_y_positive_hi=600*1.5;
    rofo.gait_part_leg[force_gait::RM].limit_y_positive_hi=600*1.5;
    rofo.gait_part_leg[force_gait::LR].limit_z_positive_hi=300;


    rofo.current_motion=ERS_RNST;
    rofo.next_motion=ERS_RNST;

    // only init here;
    // or change with IMU data
    // which we don't have currently

    rofo.bodyPeCurrent[3]=0.0;
    rofo.bodyPeCurrent[4]=0.0;
    rofo.bodyPeCurrent[5]=0.0;

    rofo.bodyPeLast[3]=0.0;
    rofo.bodyPeLast[4]=0.0;
    rofo.bodyPeLast[5]=0.0;

    rofo.BodyPeG[3]=0.0;
    rofo.BodyPeG[4]=0.0;
    rofo.BodyPeG[5]=0.0;





    cout<<"Rofo init finished"<<endl;
    return 0;

}

auto Rofo::rofoParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    CLIMB_PARAM  param;

    for(auto &i:params)
    {
        if(i.first=="count")
        {
            param.count=std::stoi(i.second);
        }
    }


    msg.copyStruct(param);
    std::cout<<"finished parse climb"<<std::endl;

}

auto Rofo::rofoGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{


    rofo.next_motion=ERS_GFWD;
    static Aris::RT_CONTROL::CMachineData data;

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const Robots::WalkParam &>(param_in);

    if(param.count==1)
    {
        rt_printf("SecondCycle\n LAST TARGE; LAST FEEDBACK; TARGET; FEEDBACK;\n");
        for(int i=0;i<18;i++)
        {
            rt_printf("%d\t%d\t%d\t%d\n",param.last_motion_raw_data->at(i).target_pos
                      ,param.last_motion_raw_data->at(i).feedback_pos
                      ,param.motion_raw_data->at(i).target_pos
                      ,param.motion_raw_data->at(i).feedback_pos);
        }

    }



    if(param.count==0)
    {
        rt_printf("FIRST CYCLE IN CLIMB\n");
        data.time=param.count;
        for(int i=0;i<18;i++)
        {
            // 2016-04-12: raw_data is already in abs layer, i think.
            data.feedbackData[i].Position=param.last_motion_raw_data->at(i).target_pos;
            data.feedbackData[i].Torque=param.motion_raw_data->at(i).feedback_cur;




            if (param.motion_raw_data->at(i).mode==aris::control::EthercatMotion::POSITION)
            {
                data.isMotorHomed[i]=true;
            }
        }
        rt_printf("LAST TARGE; LAST FEEDBACK; TARGET; FEEDBACK;\n");
        for(int i=0;i<18;i++)
        {
            rt_printf("%d\t%d\t%d\t%d\n",param.last_motion_raw_data->at(i).target_pos
                      ,param.last_motion_raw_data->at(i).feedback_pos
                      ,param.motion_raw_data->at(i).target_pos
                      ,param.motion_raw_data->at(i).feedback_pos);
        }


    }
    else
    {
        data.time=param.count;
        for(int i=0;i<18;i++)
        {
            // 2016-04-12: raw_data is already in abs layer, i think.
            // data.feedbackData[i].Position=param.motion_raw_data->at(i).feedback_pos;
            // avoid acceleration noise problem
            data.feedbackData[i].Position=param.last_motion_raw_data->at(i).target_pos;
            data.feedbackData[i].Torque=param.motion_raw_data->at(i).feedback_cur;




            if (param.motion_raw_data->at(i).mode==aris::control::EthercatMotion::POSITION)
            {
                data.isMotorHomed[i]=true;
            }
        }

    }




    if(IsRofoEnd)
    {
        rofo.next_motion=ERS_RNST;
        rofo.current_motion=ERS_RNST;
        //this step make sure RNST will run once at last
        rofo.run_gait_robot(robot,ERG_NULL,data,param);
        rt_printf("climb forced end\n");
    }
    else
    {
        // all things happen here
        rofo.run_gait_robot(robot,ERG_BACKWARD,data,param);
    }

//    if(param.count==0)
//    {

//        rt_printf("CHanges in first cycle.");
//        rt_printf("LAST TARGE; LAST FEEDBACK; TARGET; FEEDBACK;\n");
//        for(int i=0;i<18;i++)
//        {
//            rt_printf("%d\t%d\t%d\t%d\n",param.last_motion_raw_data->at(i).target_pos
//                      ,param.last_motion_raw_data->at(i).feedback_pos
//                      ,param.motion_raw_data->at(i).target_pos
//                      ,param.motion_raw_data->at(i).feedback_pos);
//        }

//    }

    if(rofo.next_motion==ERS_RNST)
    {
        rofo.current_motion=ERS_RNST;
        rt_printf("climb finished normal\n");
        IsRofoEnd=false;
        return 0;
    }
    else
    {
        return 1000;// need return a positive number, or this gait will be terminated.
    }

}

auto Rofo::ayParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    Ay_Param param;

    param.if_check_pos_min = false;
    param.if_check_pos_max = false;


    for(auto &i : params)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_leg, 6, true);
        }
        else if (i.first == "first")
        {
            param.active_leg[0] = true;
            param.active_leg[1] = false;
            param.active_leg[2] = true;
            param.active_leg[3] = false;
            param.active_leg[4] = true;
            param.active_leg[5] = false;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 0, 3, true);
            std::fill_n(param.active_motor + 6, 3, true);
            std::fill_n(param.active_motor + 12, 3, true);
        }
        else if (i.first == "second")
        {
            param.active_leg[0] = false;
            param.active_leg[1] = true;
            param.active_leg[2] = false;
            param.active_leg[3] = true;
            param.active_leg[4] = false;
            param.active_leg[5] = true;
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 3, 3, true);
            std::fill_n(param.active_motor + 9, 3, true);
            std::fill_n(param.active_motor + 15, 3, true);
        }
        else if(i.first=="ay")
        {
            double ay=stod(i.second);
            if(ay>-0.72||ay<-1.08)
            {
                ay=-0.85;
                std::cout<<"ay out of range default set to %d"<<ay<<std::endl;
            }
            param.aY=ay;
            param.IsRel=false;
        }
        else if(i.first=="dy")
        {
            param.dY=stod(i.second);
            if(param.dY<-0.3||param.dY>0.3)
            {
                param.dY=-0.05;
            }
            param.IsRel=true;
        }
        else
        {
            throw std::runtime_error("unknown param in ayParse func");
        }

    }
    msg.copyStruct(param);
}

auto Rofo::ayGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    //a duplicated function of recoverGait()
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const Ay_Param &>(param_in);


    for(int i=0;i<18;i++)
    {
        // 2016-04-12: raw_data is already in abs layer, i think.

        temp_log_data.raw_data_position[i]=param.motion_raw_data->at(i).feedback_pos;
        temp_log_data.raw_data_torque[i]=param.motion_raw_data->at(i).feedback_cur;
        temp_log_data.count=param.count;
    }
//    rt_dev_sendto(robot_log.file_id_real_time,&temp_log_data,sizeof(temp_log_data),0,NULL,0);

    static aris::server::ControlServer &cs = aris::server::ControlServer::instance();

    static double beginPee[18], endPee[18];

    if (param.count == 0)
    {
        //std::copy_n(param.motion_feedback_pos->data(), 18, beginPin);
        // get beginPee, then calculate the endPee
        robot.GetPee(beginPee, robot.body());
        robot.GetPee(endPee, robot.body());

        const double pe[6]{ 0 };
        robot.SetPeb(pe);
//        robot.SetPee(param.alignPee);
//        robot.GetPin(alignPin);
        robot.SetPee(beginPee, robot.body());

        // set endPee
        if(param.IsRel)
        {
            for(int i=0;i<6;i++)
            {
                endPee[i*3+1]+=param.dY;
            }
        }
        else
        {
            for(int i=0;i<6;i++)
            {
                endPee[i*3+1]=param.aY;
            }

        }
    }

    //int leftCount = param.count < param.recover_count ? 0 : param.recover_count;
    //int rightCount = param.count < param.recover_count ? param.recover_count : param.recover_count + param.align_count;

    double s = -(PI / 2)*cos(PI * (param.count + 1) / param.ay_count) + PI / 2;

    for (int i = 0; i < 6; ++i)
    {
        if (param.active_leg[i])
        {
            {
                double pEE[3];
                for (int j = 0; j < 3; ++j)
                {
                    pEE[j] = beginPee[i * 3 + j] * (cos(s) + 1) / 2 + endPee[i * 3 + j] * (1 - cos(s)) / 2;
                }

                robot.pLegs[i]->SetPee(pEE);
            }
        }
    }

    // recover 自己做检查 //
    for (int i = 0; i<18; ++i)
    {
        if (param.active_motor[i] && (param.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN))
        {

            if (param.motion_raw_data->at(i).target_pos >(cs.controller().motionAtAbs(i).maxPosCount() + param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is bigger than its MAX permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed\n");
                return 0;
            }
            if (param.motion_raw_data->at(i).target_pos < (cs.controller().motionAtAbs(i).minPosCount() - param.margin_offset * cs.controller().motionAtAbs(i).pos2countRatio()))
            {
                rt_printf("Motor %i's target position is smaller than its MIN permitted value in recover, you might forget to GO HOME\n", i);
                rt_printf("The min, max and current count are:\n");
                for (std::size_t i = 0; i < cs.controller().motionNum(); ++i)
                {
                    rt_printf("%d   %d   %d\n", cs.controller().motionAtAbs(i).minPosCount(), cs.controller().motionAtAbs(i).maxPosCount(), param.motion_raw_data->at(i).target_pos);
                }
                rt_printf("recover failed\n");
                return 0;
            }
        }
    }

    return param.ay_count - param.count - 1;

}


//int adjustY(Robots::RobotBase * pRobot, const Aris::Server::GaitParamBase * pParam)
//{
////    rt_printf("%d run adjustY\n",pParam->count);
//    auto pAP0 = static_cast<const ADY_PARAM*>(pParam);




//    // modify targetPee[0] here becasue we only have one period here.
//    if(pAP->count==0)
//    {
//        rt_printf("init adjustY\n");
//        memcpy(pAP,pAP0,sizeof(ADY_PARAM));
//        rt_printf("finish copy param\n");
//        if(pAP->IsRel)
//        {
//            // Relative
//            for(int i=0;i<18;i++)
//            {
//                pAP->targetPee[0][i]=pAP->beginPee[i];
//                if(i%3==1)
//                {
//                    pAP->targetPee[0][i]+=pAP->dY;
//                }
//                adjustYTargetPee[i]=pAP->targetPee[0][i];
//            }
//        }
//        else
//        {
//            //Abs
//            for(int i=0;i<18;i++)
//            {
//                pAP->targetPee[0][i]=pAP->beginPee[i];
//                if(i%3==1)
//                {
//                    pAP->targetPee[0][i]=pAP->aY;
//                }
//                adjustYTargetPee[i]=pAP->targetPee[0][i];
//            }
//        }

//    }
//    else
//    {
//        memcpy(pAP,pAP0,sizeof(ADY_PARAM));
//        for(int i=0;i<18;i++)
//        {
//            pAP->targetPee[0][i]=adjustYTargetPee[i];
//        }

//    }


//    int pos, periodBeginCount{ 0 }, periodEndCount{ 0 };
//    double realTargetPee[ADY_PARAM::MAX_PERIOD_NUM][18];
//    double realTargetPbody[ADY_PARAM::MAX_PERIOD_NUM][6];

//    /*转换末端和身体目标位置的坐标到地面坐标系下*/
//    for (int i = 0; i < pAP->periodNum; ++i)
//    {
//        pRobot->TransformCoordinatePee(pAP->beginBodyPE, pAP->relativeCoordinate, pAP->targetPee[i], "G",realTargetPee[i]);
//    }

//    switch (*(pAP->relativeBodyCoordinate))
//    {
//    case 'B':
//    case 'M':
//    {
//        double beginPm[16];
//        Aris::DynKer::s_pe2pm(pAP->beginBodyPE, beginPm);

//        for (int i = 0; i < pAP->periodNum; ++i)
//        {
//            double pm1[16], pm2[16];

//            Aris::DynKer::s_pe2pm(pAP->targetBodyPE[i], pm1);
//            Aris::DynKer::s_pm_dot_pm(beginPm, pm1, pm2);
//            Aris::DynKer::s_pm2pe(pm2, realTargetPbody[i]);
//        }
//        break;
//    }

//    case 'G':
//    case 'O':
//    default:
//        std::copy_n(*pAP->targetBodyPE, ADY_PARAM::MAX_PERIOD_NUM*6, *realTargetPbody);
//        break;
//    }

//    /*判断当前所处的周期*/
//    for (int i = 0; i < pAP->periodNum; ++i)
//    {
//        periodEndCount += pAP->periodCount[i];

//        if ((pAP->count < periodEndCount) && (pAP->count >= periodBeginCount))
//        {
//            pos = i;
//            break;
//        }

//        periodBeginCount = periodEndCount;
//    }

//    double s = -(PI / 2)*cos(PI * (pAP->count - periodBeginCount + 1) / (periodEndCount- periodBeginCount)) + PI / 2;

//    /*插值当前的末端和身体位置*/
//    double pEE[18], pBody[6];

//    if (pos == 0)
//    {
//        for (int i = 0; i < 18; ++i)
//        {
//            pEE[i] = pAP->beginPee[i] * (cos(s) + 1) / 2 + realTargetPee[pos][i] * (1 - cos(s)) / 2;

//        }
//        for (int i = 0; i < 6; ++i)
//        {
//            pBody[i] = pAP->beginBodyPE[i] * (cos(s) + 1) / 2 + realTargetPbody[pos][i] * (1 - cos(s)) / 2;
//        }
//    }
//    else
//    {
//        for (int i = 0; i < 18; ++i)
//        {
//            pEE[i] = realTargetPee[pos-1][i] * (cos(s) + 1) / 2 + realTargetPee[pos][i] * (1 - cos(s)) / 2;

//        }
//        for (int i = 0; i < 6; ++i)
//        {
//            pBody[i] = realTargetPbody[pos - 1][i] * (cos(s) + 1) / 2 + realTargetPbody[pos][i] * (1 - cos(s)) / 2;
//        }
//    }

//    pRobot->SetPee(pEE, pBody);

//    /*计算总共所需要花的时间，以便返回剩余的count数*/
//    int totalCount{ 0 };
//    for (int i = 0; i < pAP->periodNum; ++i)
//    {
//        totalCount += pAP->periodCount[i];
//    }


//    return totalCount - pAP->count - 1;
//}


//Aris::Core::MSG parseDig(const std::string &cmd, const map<std::string, std::string> &params)
//{
//    double firstEE[18] =
//    {
//        -0.3,-0.75,-0.65,
//        -0.45,-0.75,0,
//        -0.3,-0.75,0.65,
//        0.3,-0.75,-0.65,
//        0.45,-0.75,0,
//        0.3,-0.75,0.65,
//    };

//    double beginEE[18]
//    {
//        -0.3,-0.85,-0.65,
//        -0.45,-0.85,0,
//        -0.3,-0.85,0.65,
//        0.3,-0.85,-0.65,
//        0.45,-0.85,0,
//        0.3,-0.85,0.65,
//    };

//    double PointPee1[18]
//    {
//        -0.3,-0.85,-0.65,
//        -0.45,-0.85,0,
//        -0.3,-0.85,0.65,
//        0.3,-0.85,-0.65,
//        0.45,-0.85,0,
//        0.3,-0.85,0.65,
//    };
//    double PointBodyPe1[6]
//    {
//        0,0,-0.1,
//        0,5.0/180.0*3.1415926,0
//    };

//    double PointBodyPe2[6]
//    {
//        0,-0.08,-0.1,
//        0,5.0/180.0*3.1415926,0
//    };

//    double PointBodyPe3[6]
//    {
//        0,-0.08,0,
//        0,5.0/180.0*3.1415926,0
//    };

//    double PointBodyPe4[6]
//    {
//        0,0,0,
//        0,0,0
//    };

//    double depth=-0.08;
//    for (auto &i : params)
//    {
//        if (i.first == "depth")
//        {
//            depth=stod(i.second);

//        }
//    }
//    PointBodyPe2[1]=depth;
//    PointBodyPe3[1]=depth;




//    Robots::ADJUST_PARAM  param;

////    std::copy_n(firstEE, 18, param.targetPee[0]);
////    std::fill_n(param.targetBodyPE[0], 6, 0);
////    std::copy_n(beginEE, 18, param.targetPee[1]);
////    std::fill_n(param.targetBodyPE[1], 6, 0);

//    std::copy_n(PointPee1,18,param.targetPee[0]);
//    std::copy_n(PointPee1,18,param.targetPee[1]);
//    std::copy_n(PointPee1,18,param.targetPee[2]);
//    std::copy_n(PointPee1,18,param.targetPee[3]);

//    std::copy_n(PointBodyPe1,6,param.targetBodyPE[0]);
//    std::copy_n(PointBodyPe2,6,param.targetBodyPE[1]);
//    std::copy_n(PointBodyPe3,6,param.targetBodyPE[2]);
//    std::copy_n(PointBodyPe4,6,param.targetBodyPE[3]);



//    param.periodNum = 4;
//    param.periodCount[0] = 5000;
//    param.periodCount[1] = 5000;
//    param.periodCount[2] = 5000;
//    param.periodCount[3] = 5000;

//    std::strcpy(param.relativeCoordinate, "B");
//    std::strcpy(param.relativeBodyCoordinate, "B");


//    Aris::Core::MSG msg;

//    msg.CopyStruct(param);

//    return msg;
//}


//int Dig(Robots::RobotBase * pRobot, const Aris::Server::GaitParamBase * pParam)
//{
//    auto pAP = static_cast<const Robots::ADJUST_PARAM*>(pParam);

//    int pos{0}, periodBeginCount{0}, periodEndCount{0};
//    double realTargetPee[Robots::ADJUST_PARAM::MAX_PERIOD_NUM][18];
//    double realTargetPbody[Robots::ADJUST_PARAM::MAX_PERIOD_NUM][6];

//    /*转换末端和身体目标位置的坐标到地面坐标系下*/
//    for (int i = 0; i < pAP->periodNum; ++i)
//    {
//        pRobot->TransformCoordinatePee(pAP->beginBodyPE, pAP->relativeCoordinate, pAP->targetPee[i], "G",realTargetPee[i]);
//    }

//    switch (*(pAP->relativeBodyCoordinate))
//    {
//    case 'B':
//    case 'M':
//    {
//        double beginPm[16];
//        Aris::DynKer::s_pe2pm(pAP->beginBodyPE, beginPm);

//        for (int i = 0; i < pAP->periodNum; ++i)
//        {
//            double pm1[16], pm2[16];

//            Aris::DynKer::s_pe2pm(pAP->targetBodyPE[i], pm1);
//            Aris::DynKer::s_pm_dot_pm(beginPm, pm1, pm2);
//            Aris::DynKer::s_pm2pe(pm2, realTargetPbody[i]);
//        }
//        break;
//    }

//    case 'G':
//    case 'O':
//    default:
//        std::copy_n(&pAP->targetBodyPE[0][0], Robots::ADJUST_PARAM::MAX_PERIOD_NUM*6, &realTargetPbody[0][0]);
//        break;
//    }

//    /*判断当前所处的周期*/
//    for (int i = 0; i < pAP->periodNum; ++i)
//    {
//        periodEndCount += pAP->periodCount[i];

//        if ((pAP->count < periodEndCount) && (pAP->count >= periodBeginCount))
//        {
//            pos = i;
//            break;
//        }

//        periodBeginCount = periodEndCount;
//    }

//    double s = -(PI / 2)*cos(PI * (pAP->count - periodBeginCount + 1) / (periodEndCount- periodBeginCount)) + PI / 2;

//    /*插值当前的末端和身体位置*/
//    double pEE[18], pBody[6];

//    if (pos == 0)
//    {
//        for (int i = 0; i < 18; ++i)
//        {
//            pEE[i] = pAP->beginPee[i] * (cos(s) + 1) / 2 + realTargetPee[pos][i] * (1 - cos(s)) / 2;
//        }

//        /*以下用四元数进行插值*/
//        double pq_first[7], pq_second[7], pq[7];
//        Aris::DynKer::s_pe2pq(pAP->beginBodyPE, pq_first);
//        Aris::DynKer::s_pe2pq(realTargetPbody[pos], pq_second);

//        if (Aris::DynKer::s_vn_dot_vn(4, &pq_first[3], &pq_second[3]) < 0)
//        {
//            for (int i = 3; i < 7; ++i)
//            {
//                pq_second[i] = -pq_second[i];
//            }
//        }

//        for (int i = 0; i < 7; ++i)
//        {
//            pq[i] = pq_first[i] * (cos(s) + 1) / 2 + pq_second[i] * (1 - cos(s)) / 2;
//        }
//        Aris::DynKer::s_pq2pe(pq, pBody);
//    }
//    else
//    {
//        for (int i = 0; i < 18; ++i)
//        {
//            pEE[i] = realTargetPee[pos-1][i] * (cos(s) + 1) / 2 + realTargetPee[pos][i] * (1 - cos(s)) / 2;

//        }
//        /*for (int i = 0; i < 6; ++i)
//        {
//            pBody[i] = realTargetPbody[pos - 1][i] * (cos(s) + 1) / 2 + realTargetPbody[pos][i] * (1 - cos(s)) / 2;
//        }*/
//        /*以下用四元数进行插值*/
//        double pq_first[7], pq_second[7], pq[7];
//        Aris::DynKer::s_pe2pq(realTargetPbody[pos - 1], pq_first);
//        Aris::DynKer::s_pe2pq(realTargetPbody[pos], pq_second);

//        if (Aris::DynKer::s_vn_dot_vn(4, &pq_first[3], &pq_second[3]) < 0)
//        {
//            for (int i = 3; i < 7; ++i)
//            {
//                pq_second[i] = -pq_second[i];
//            }
//        }

//        for (int i = 0; i < 7; ++i)
//        {
//            pq[i] = pq_first[i] * (cos(s) + 1) / 2 + pq_second[i] * (1 - cos(s)) / 2;
//        }
//        Aris::DynKer::s_pq2pe(pq, pBody);
//    }

//    pRobot->SetPee(pEE, pBody);

//    /*计算总共所需要花的时间，以便返回剩余的count数*/
//    int totalCount{ 0 };
//    for (int i = 0; i < pAP->periodNum; ++i)
//    {
//        totalCount += pAP->periodCount[i];
//    }


//    return totalCount - pAP->count - 1;

//}
