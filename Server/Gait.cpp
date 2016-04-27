#include"Gait.h"


//double ForceGait::CGaitRobot::RotL2M[6][3][3];
//double ForceGait::CGaitRobot::TranL2M[6][4][4];

peripherals::Log robot_log;
LogData temperary_log_data;

char LEG_NAME[][3]={"LF","LM","LB","RF","RM","RB"};
char GAIT_NAME[][80]={
    "GAIT_NONE",
    "GAIT_POWEROFF",
    "GAIT_STOP",
    "GAIT_ENABLE",
    "GAIT_HOME",
    "GAIT_H2S",
    "GAIT_STANDSTILL",
    "GAIT_MAJOR_UP",
    "GAIT_MINOR_UP",
    "GAIT_MAJOR_FORWARD",
    "GAIT_MINOR_FORWARD",
    "GAIT_MAJOR_BACKWARD",
    "GAIT_MINOR_BACKWARD",
    "GAIT_MAJOR_DOWN",
    "GAIT_MINOR_DOWN",
    "GAIT_MINOR_UP_BACKWARD",
    "GAIT_STRAIGHT",
    "GAIT_LOCAL_CYCLE_FOREWARD",
    "GAIT_LOCAL_CYCLE_BACKWARD",
    "GAIT_REFLEX",
    "GAIT_ELLIPSE"
};

force_gait::EMOTION force_gait::GaitRobot::transition_matrix[TM_SIZE][TM_SIZE];

// Threshold
force_gait::Threshold::Threshold()
{
    this->hi_thr=600;
    this->lo_thr=100;
    is_on_=false;
}

force_gait::Threshold::~Threshold()
{

}
void force_gait::Threshold::reset()
{
    is_on_=false;
}

bool force_gait::Threshold::threshold(double val)
{
    this->value=val;
    if(is_on_)
    {
        if(val<lo_thr)
        {
            is_on_=false;
        }

    }
    else
    {
        if(val>hi_thr)
        {
            is_on_=true;
        }

    }
    return is_on_;
}
void force_gait::Threshold::set_threshold(double lo, double hi)
{
    this->lo_thr=lo;
    this->hi_thr=hi;
}

bool force_gait::Threshold::is_on()
{
    return is_on_;
}

force_gait::AbstractMotorData& force_gait::AbstractMotorData::operator =(const AbstractMotorData& other)
{
    memcpy(this,&other,sizeof(*this));
    return *this;
}

//CGaitPart is a template class, definition should be put in the header.
//CGaitRobot
force_gait::GaitRobot::GaitRobot()
{
    for(int i=0;i<6;i++)
    {
        leg_status[i].leg_id=i;
        gait_part_leg[i].id=i;
        bodyPeCurrent[i]=0;
        bodyPeLast[i]=0;
        BodyPeG[i]=0;

    };


    gait_part_leg[LF].limb_id[0]=10;
    gait_part_leg[LF].limb_id[1]=11;
    gait_part_leg[LF].limb_id[2]=9;

    gait_part_leg[LM].limb_id[0]=12;
    gait_part_leg[LM].limb_id[1]=14;
    gait_part_leg[LM].limb_id[2]=13;

    gait_part_leg[LR].limb_id[0]=17;
    gait_part_leg[LR].limb_id[1]=15;
    gait_part_leg[LR].limb_id[2]=16;

    gait_part_leg[RF].limb_id[0]=6;
    gait_part_leg[RF].limb_id[1]=8;
    gait_part_leg[RF].limb_id[2]=7;

    gait_part_leg[RM].limb_id[0]=3;
    gait_part_leg[RM].limb_id[1]=5;
    gait_part_leg[RM].limb_id[2]=4;

    gait_part_leg[RR].limb_id[0]=0;
    gait_part_leg[RR].limb_id[1]=2;
    gait_part_leg[RR].limb_id[2]=1;


    // set up correct AbsID for each leg



    //    //set TransitionMatrix
    //    //MO_NONE
    //    TransitionMatrix[EOC_NONE][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_H2S][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_FORWARD][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_STOP][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_STANDSTILL][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_X][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_Y][MO_NONE]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_Z][MO_NONE]=MO_STANDSTILL;

    //    //MO_STANDSTILL
    //    TransitionMatrix[EOC_NONE][MO_STANDSTILL]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_H2S][MO_STANDSTILL]=MO_H2S;
    //    TransitionMatrix[EOC_FORWARD][MO_STANDSTILL]=MO_FORWARD;
    //    TransitionMatrix[EOC_STOP][MO_STANDSTILL]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_STANDSTILL]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_STANDSTILL]=MO_CA_S;
    //    TransitionMatrix[EOC_CA_X][MO_STANDSTILL]=MO_CA_X;
    //    TransitionMatrix[EOC_CA_Y][MO_STANDSTILL]=MO_CA_Y;
    //    TransitionMatrix[EOC_CA_Z][MO_STANDSTILL]=MO_CA_Z;

    //    TransitionMatrix[EOC_CA_BS][MO_STANDSTILL]=MO_CA_BS;
    //    TransitionMatrix[EOC_CA_BX][MO_STANDSTILL]=MO_CA_BX;
    //    TransitionMatrix[EOC_CA_BY][MO_STANDSTILL]=MO_CA_BY;
    //    TransitionMatrix[EOC_CA_BZ][MO_STANDSTILL]=MO_CA_BZ;
    //    TransitionMatrix[EOC_CA_BRX][MO_STANDSTILL]=MO_CA_BRX;
    //    TransitionMatrix[EOC_CA_BRY][MO_STANDSTILL]=MO_CA_BRY;
    //    TransitionMatrix[EOC_CA_BRZ][MO_STANDSTILL]=MO_CA_BRZ;



    //    //MO_H2S
    //    TransitionMatrix[EOC_NONE][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_H2S][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_FORWARD][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_STOP][MO_H2S]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_CA_S][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_CA_X][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_CA_Y][MO_H2S]=MO_H2S;
    //    TransitionMatrix[EOC_CA_Z][MO_H2S]=MO_H2S;

    //    //MO_FORWARD
    //    TransitionMatrix[EOC_NONE][MO_FORWARD]=MO_FORWARD;
    //    TransitionMatrix[EOC_H2S][MO_FORWARD]=MO_FORWARD;
    //    TransitionMatrix[EOC_FORWARD][MO_FORWARD]=MO_FORWARD;
    //    TransitionMatrix[EOC_STOP][MO_FORWARD]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_FORWARD]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_FORWARD]=MO_FORWARD;
    //    TransitionMatrix[EOC_CA_X][MO_FORWARD]=MO_FORWARD;
    //    TransitionMatrix[EOC_CA_Y][MO_FORWARD]=MO_FORWARD;
    //    TransitionMatrix[EOC_CA_Z][MO_FORWARD]=MO_FORWARD;
    //    //MO_STOP
    //    TransitionMatrix[EOC_NONE][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_H2S][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_FORWARD][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_STOP][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_CA_S][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_CA_X][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_CA_Y][MO_STOP]=MO_STOP;
    //    TransitionMatrix[EOC_CA_Z][MO_STOP]=MO_STOP;
    //    //MO_CA_S
    //    TransitionMatrix[EOC_NONE][MO_CA_S]=MO_CA_S;
    //    TransitionMatrix[EOC_H2S][MO_CA_S]=MO_CA_S;
    //    TransitionMatrix[EOC_FORWARD][MO_CA_S]=MO_CA_S;
    //    TransitionMatrix[EOC_STOP][MO_CA_S]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_CA_S]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_CA_S]=MO_CA_S;
    //    TransitionMatrix[EOC_CA_X][MO_CA_S]=MO_CA_S;
    //    TransitionMatrix[EOC_CA_S][MO_CA_S]=MO_CA_S;
    //    TransitionMatrix[EOC_CA_X][MO_CA_S]=MO_CA_S;
    //    //MO_CA_X
    //    TransitionMatrix[EOC_NONE][MO_CA_X]=MO_CA_X;
    //    TransitionMatrix[EOC_H2S][MO_CA_X]=MO_CA_X;
    //    TransitionMatrix[EOC_FORWARD][MO_CA_X]=MO_CA_X;
    //    TransitionMatrix[EOC_STOP][MO_CA_X]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_CA_X]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_CA_X]=MO_CA_X;
    //    TransitionMatrix[EOC_CA_X][MO_CA_X]=MO_CA_X;
    //    TransitionMatrix[EOC_CA_Y][MO_CA_X]=MO_CA_X;
    //    TransitionMatrix[EOC_CA_Z][MO_CA_X]=MO_CA_X;
    //    //MO_CA_Y
    //    TransitionMatrix[EOC_NONE][MO_CA_Y]=MO_CA_Y;
    //    TransitionMatrix[EOC_H2S][MO_CA_Y]=MO_CA_Y;
    //    TransitionMatrix[EOC_FORWARD][MO_CA_Y]=MO_CA_Y;
    //    TransitionMatrix[EOC_STOP][MO_CA_Y]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_CA_Y]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_CA_Y]=MO_CA_Y;
    //    TransitionMatrix[EOC_CA_X][MO_CA_Y]=MO_CA_Y;
    //    TransitionMatrix[EOC_CA_Y][MO_CA_Y]=MO_CA_Y;
    //    TransitionMatrix[EOC_CA_Z][MO_CA_Y]=MO_CA_Y;

    //    //MO_CA_Z
    //    TransitionMatrix[EOC_NONE][MO_CA_Z]=MO_CA_Z;
    //    TransitionMatrix[EOC_H2S][MO_CA_Z]=MO_CA_Z;
    //    TransitionMatrix[EOC_FORWARD][MO_CA_Z]=MO_CA_Z;
    //    TransitionMatrix[EOC_STOP][MO_CA_Z]=MO_STOP;
    //    TransitionMatrix[EOC_STANDSTILL][MO_CA_Z]=MO_STANDSTILL;
    //    TransitionMatrix[EOC_CA_S][MO_CA_Z]=MO_CA_Z;
    //    TransitionMatrix[EOC_CA_X][MO_CA_Z]=MO_CA_Z;
    //    TransitionMatrix[EOC_CA_Y][MO_CA_Z]=MO_CA_Z;
    //    TransitionMatrix[EOC_CA_Z][MO_CA_Z]=MO_CA_Z;


    robotStateMachine.SetNames(ERG_NAMES,ERGS_NAMES);
    robotStateMachine.SetOffset(EROFOGAIT_OFFSET,EROFOGAITSTA_OFFSET);
    //RobotSM.SetOffset(10,20);
    for(int i=0;i<ERGS_NUM;i++)
    {
        robotStateMachine.AddState(ERofoGaitSTA(i+EROFOGAITSTA_OFFSET));
    }
    robotStateMachine.AddOrderWithState(ERofoGait::ERG_POWEROFF,
                                        ERofoGaitSTA::ERS_POWEROFF);
    robotStateMachine.AddOrderWithState(ERofoGait::ERG_STOP,
                                        ERofoGaitSTA::ERS_STOP);
    robotStateMachine.AddOrderWithState(ERofoGait::ERG_ENABLE,
                                        ERofoGaitSTA::ERS_ENABLE);

    //    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_ENABLE,
    //                                    ERofoGait::ERG_RUNNING,
    //                                    ERofoGaitSTA::ERS_RUNNING);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_ENABLED,
                                    ERofoGait::ERG_RUNNING,
                                    ERofoGaitSTA::ERS_RUNNING);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_FORWARD,
                                    ERofoGaitSTA::ERS_GFWD);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_BACKWARD,
                                    ERofoGaitSTA::ERS_GBWD);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_LEFT,
                                    ERofoGaitSTA::ERS_GLFT);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_RIGHT,
                                    ERofoGaitSTA::ERS_GRGT);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_EXPRI1,
                                    ERofoGaitSTA::ERS_GEX1);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RUNNING,
                                    ERofoGait::ERG_HOME1,
                                    ERofoGaitSTA::ERS_HME1);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RUNNING,
                                    ERofoGait::ERG_HOME2,
                                    ERofoGaitSTA::ERS_HME2);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RUNNING,
                                    ERofoGait::ERG_ENABLE,
                                    ERofoGaitSTA::ERS_ENABLE);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_HM2D,
                                    ERofoGait::ERG_HOME1,
                                    ERofoGaitSTA::ERS_HME1);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_HM1D,
                                    ERofoGait::ERG_HOME2,
                                    ERofoGaitSTA::ERS_HME2);

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_HM1D,
                                    ERofoGait::ERG_H2ST1,
                                    ERofoGaitSTA::ERS_H2S1);//switch to ERS_HM1D again or ERS_RNST

    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_HM2D,// HM2D means these motors are running
                                    ERofoGait::ERG_H2ST2,
                                    ERofoGaitSTA::ERS_H2S2);

    // Temporary solution for two homing groups
    // Two following transitions
    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_H2ST1,
                                    ERofoGaitSTA::ERS_H2S1);
    robotStateMachine.AddTransition(ERofoGaitSTA::ERS_RNST,
                                    ERofoGait::ERG_H2ST2,
                                    ERofoGaitSTA::ERS_H2S2);



    robotStateMachine.SetInitialState(ERofoGaitSTA::ERS_NULL);
    //    robotStateMachine.PrintStateMachine();


}
force_gait::GaitRobot::~GaitRobot()
{

}
void force_gait::GaitRobot::init()
{
    this->get_transform_matrix_leg_to_body(force_gait::GaitRobot::RotL2M[LF],
                                           force_gait::GaitRobot::TranL2M[LF],
                                           LF);
    this->get_transform_matrix_leg_to_body(force_gait::GaitRobot::RotL2M[LM],
                                           force_gait::GaitRobot::TranL2M[LM],
                                           LM);
    this->get_transform_matrix_leg_to_body(force_gait::GaitRobot::RotL2M[LR],
                                           force_gait::GaitRobot::TranL2M[LR],
                                           LR);
    this->get_transform_matrix_leg_to_body(force_gait::GaitRobot::RotL2M[RF],
                                           force_gait::GaitRobot::TranL2M[RF],
                                           RF);
    this->get_transform_matrix_leg_to_body(force_gait::GaitRobot::RotL2M[RM],
                                           force_gait::GaitRobot::TranL2M[RM],
                                           RM);
    this->get_transform_matrix_leg_to_body(force_gait::GaitRobot::RotL2M[RR],
                                           force_gait::GaitRobot::TranL2M[RR],
                                           RR);

    for(int i=0;i<6;i++)
    {
        // used for stepping down edge, hope it works
        // test this after step up is OK
        // when it is off, it means the leg has a big negative force
        this->gait_part_leg[i].threshold_z_negative.set_threshold(-150.0,-50.0);
    }
}

void force_gait::GaitRobot::get_transform_matrix_leg_to_body(double rotx[][3],double trans[][4], const force_gait::ELEGID LegID)
{
    //static double rot1[3][3],rot2[3][3],rot3[3][3],rot[3][3],tran[4][4];
    double ep[6];
    static double ag[3],pos[3];
    switch(LegID)
    {
    case LF:
    {
        //Aris::Dynamic::s_pm2pe(this->Robot.pLF->pBase->GetPrtPmPtr(),ep);
        auto init_temp = std::initializer_list<double>({-0.0679295 , 0 , -0.3552145, PI/2,  PI* 5/6,   -PI/2-PI*7/18});
        std::copy(init_temp.begin(), init_temp.end(), ep);
        break;
    }
    case LM:
    {
        //Aris::Dynamic::s_pm2pe(this->Robot.pLF->pBase->GetPrtPmPtr(),ep);
        auto init_temp = std::initializer_list<double>({-0.1234141 , 0 ,  0,         PI/2,  PI* 6/6,   -PI/2-PI*7/18});
        std::copy(init_temp.begin(), init_temp.end(), ep);
        break;
    }
    case LR:
    {
        //Aris::Dynamic::s_pm2pe(this->Robot.pLF->pBase->GetPrtPmPtr(),ep);
        auto init_temp = std::initializer_list<double>({-0.0679295 , 0 ,  0.3532904, PI/2,  PI* 7/6,   -PI/2-PI*7/18});
        std::copy(init_temp.begin(), init_temp.end(), ep);
        break;
    }
    case RF:
    {
        //Aris::Dynamic::s_pm2pe(this->Robot.pLF->pBase->GetPrtPmPtr(),ep);
        auto init_temp = std::initializer_list<double>({ 0.0679295 , 0 , -0.3552145, PI/2,  PI* 1/6,   -PI/2-PI*7/18});
        std::copy(init_temp.begin(), init_temp.end(), ep);
        break;
    }
    case RM:
    {
        //Aris::Dynamic::s_pm2pe(this->Robot.pLF->pBase->GetPrtPmPtr(),ep);
        auto init_temp = std::initializer_list<double>({ 0.1234141 , 0 ,  0,         PI/2,  PI* 0/6,   -PI/2-PI*7/18});
        std::copy(init_temp.begin(), init_temp.end(), ep);
        break;
    }
    case RR:
    {
        //Aris::Dynamic::s_pm2pe(this->Robot.pLF->pBase->GetPrtPmPtr(),ep);

        auto init_temp = std::initializer_list<double>({ 0.0679295 , 0 ,  0.3532904, PI/2,  PI*11/6,   -PI/2-PI*7/18});
        std::copy(init_temp.begin(), init_temp.end(), ep);
        break;
    }
    default:
        break;
    }


    ag[0]=ep[3];
    ag[1]=ep[5];
    ag[2]=ep[5];
    pos[0]=ep[0];
    pos[1]=ep[1];
    pos[2]=ep[2];
    printf("ag[0] %.5f ag[1] %.5f ag[2] %.5f ",ag[0],ag[1],ag[2]);
    printf("pos[0] %.5f pos[1] %.5f pos[2] %.5f\n",pos[0],pos[1],pos[2]);




    double rot1[3][3]=
    {
        {cos(ag[0]),-1.0*sin(ag[0]),0},
        {sin(ag[0]),cos(ag[0]),     0},
        {0,         0,              1}
    };

    double rot2[3][3]=
    {
        {1,         0,              0},
        {0,cos(ag[1]),-1.0*sin(ag[1])},
        {0,sin(ag[1]),      cos(ag[1])}
    };

    double rot3[3][3]=
    {
        {cos(ag[2]),-1.0*sin(ag[2]),0},
        {sin(ag[2]),cos(ag[2]),     0},
        {0,         0,              1}
    };
    double rot[3][3];
    aris::dynamic::s_dgemm(3,3,3,1.0,*rot1,3,*rot2,3,0,*rot,3);
    aris::dynamic::s_dgemm(3,3,3,1.0,*rot,3,*rot3,3,0,*rotx,3);

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            trans[i][j]=rotx[i][j];
        }
    }
    trans[3][0]=0;
    trans[3][1]=0;
    trans[3][2]=0;
    trans[3][3]=1;
    trans[0][3]=pos[0];
    trans[1][3]=pos[1];
    trans[2][3]=pos[2];

    std::cout<<"Finished"<<endl;

}


void force_gait::GaitRobot::motor_data_to_model_data(Aris::RT_CONTROL::CMotorData* inData,
                                                     Aris::RT_CONTROL::CMotorData* inDataLast,
                                                     force_gait::AbstractMotorData* outData,
                                                     AbstractMotorData* outDataLast)
{

    for(int i=0;i<MOTORS_NUM;i++)
    {
        //position
        outData[i].position=((double)inData[i].Position)/RATIO;//m
        outData[i].velocity=(double)(inData[i].Position-inDataLast[i].Position)/RATIO*1000.0;//m/s
        outData[i].acceleration=(outData[i].velocity-outDataLast[i].velocity)*1000.0;//m/s/s
        /*
        outData[i].Force=(inData[i].Torque/1000.0*9.38*84.9/1000-
                          outData[i].Acceleration*RATIO/65536*2*3.1415926*RotorInertia)
                *2*3.1415926/(10.0/1000)*3.5;//Newton
        */

        //without motor inertia effect
        outData[i].force=(inData[i].Torque/1000.0*9.38*84.9/1000)
                *2*3.1415926/(10.0/1000)*3.5;//Newton
    }



    for(int i=0;i<MOTORS_NUM;i++)
    {
        inDataLast[i]=inData[i];//MotorData
        outDataLast[i]=outData[i];//AbstractMotorData
    }


}

void force_gait::GaitRobot::model_data_to_motor_data(force_gait::AbstractMotorData* inData,
                                                     Aris::RT_CONTROL::CMotorData* outData)
{
    for(int i=0;i<MOTORS_NUM;i++)
    {
        outData[i].Position=(int)(inData[i].position*RATIO);
        outData[i].Velocity=(int)(inData[i].velocity*RATIO);
        outData[i].Torque=(int)(inData[i].force);//!!!!!!!!!!!!!!!!!!!!Not usable
    }

}

void force_gait::GaitRobot::get_standstill_command_data(Aris::RT_CONTROL::CMachineData &data)
{
    for(int i=0;i<18;i++)
    {
        data.commandData[i]=this->standstill_machine_data.commandData[i];
        data.isMotorHomed[i]=this->standstill_machine_data.isMotorHomed[i];
        data.motorsCommands[i]=this->standstill_machine_data.motorsCommands[i];
        data.motorsModes[i]=this->standstill_machine_data.motorsModes[i];
    }
    //    data.IsLogging=this->standStillMachineData.IsLogging;

}

void force_gait::GaitRobot::filter_model_data(AbstractMotorData *inData, AbstractMotorData *outData)
{
    for(int i=0;i<MOTORS_NUM;i++)
    {
        this->acceleration_filter[i].Filter(inData[i].acceleration,outData[i].acceleration);
        this->position_filter[i].Filter(inData[i].position,outData[i].position);
        this->velocity_filter[i].Filter(inData[i].velocity,outData[i].velocity);
        this->force_filter[i].Filter(inData[i].force,outData[i].force);
    }

}


void force_gait::GaitRobot::model_evaluation(Robots::RobotTypeI& robot,aris::dynamic::FloatMarker &beginMak,Aris::RT_CONTROL::CMachineData &data)
{
    this->machine_data_in = data;
    for(int i=0;i<MOTORS_NUM;i++)
    {
        this->feedback_motor_data[i] = data.feedbackData[i];

        temperary_log_data.raw_data_position[i]=data.feedbackData[i].Position;
        temperary_log_data.raw_data_torque[i]=data.feedbackData[i].Torque;
    }

    for(int i=0;i<MOTORS_NUM;i++)
    {
        //        this->m_feedbackMotorDataMapped[i]=data.feedbackData[MapAbsToPhy[i]];
        this->feedback_motor_data_mapped[i]=data.feedbackData[i];
    }
    this->motor_data_to_model_data(this->feedback_motor_data_mapped,
                                   this->feedback_motor_data_mapped_last,
                                   this->feedback_model_data_mapped,
                                   this->feedback_model_data_mapped_last);

    if(this->current_motion==ERS_RNST||data.time==0)
    {
        rt_printf("Prepare filters\n");
        for(int i=0;i<18;i++)
        {
            this->acceleration_filter[i].ResetReg();
            this->position_filter[i].ResetReg();
            this->velocity_filter[i].ResetReg();
            this->force_filter[i].ResetReg();
        }
        for(int i=0;i<100;i++)
        {
            this->filter_model_data(this->feedback_model_data_mapped,
                                    this->feedback_model_data_mapped_filtered);
        }
        //        rt_printf("First ModelCalculation %d\n",data.time);
        //        rt_printf("first p pos:%f\n",this->m_feedbackModelDataMappedFiltered[0].Postion);
    }

    this->filter_model_data(this->feedback_model_data_mapped,
                            this->feedback_model_data_mapped_filtered);

    this->online_angle[0]=3.14159265357;
    //this->online_angle[0]=0.0;
    // IMU2R
    static double pm_IMU2G0[4][4];
    aris::dynamic::s_pe2pm(this->online_angle,*pm_IMU2G0,"321");
    static double pm_IMU2R[4][4]=
    {{1,0,0,0},
     {0,0,-1,0},
     {0,1,0,0},
     {0,0,0,1}};
    static double pm_R2IMU[4][4]=
    {{1,0,0,0},
     {0,0,1,0},
     {0,-1,0,0},
     {0,0,0,1}};
    static double pm_G02G[4][4]=
    {
        {-1,0,0,0},
        {0,0,1,0},
        {0,1,0,0},
        {0,0,0,1},
    };
    static double pm_RG0[4][4];
    static double pm_RG0_tmp[4][4];
    //    Aris::Dynamic::s_pm_dot_pm(*pm_IMU2G0,*pm_R2IMU,*pm_RG0);
    aris::dynamic::s_pm_dot_pm(*pm_IMU2G0,*pm_R2IMU,*pm_RG0_tmp);
    aris::dynamic::s_pm_dot_pm(*pm_G02G,*pm_RG0_tmp,*pm_RG0);


    //use filtered data to calculate forces
    this->leg_status[LF].prismatic_actuation_force[0]=this->feedback_model_data_mapped_filtered[0].force;
    this->leg_status[LF].prismatic_actuation_force[1]=this->feedback_model_data_mapped_filtered[1].force;
    this->leg_status[LF].prismatic_actuation_force[2]=this->feedback_model_data_mapped_filtered[2].force;

    this->leg_status[LM].prismatic_actuation_force[0]=this->feedback_model_data_mapped_filtered[3].force;
    this->leg_status[LM].prismatic_actuation_force[1]=this->feedback_model_data_mapped_filtered[4].force;
    this->leg_status[LM].prismatic_actuation_force[2]=this->feedback_model_data_mapped_filtered[5].force;

    this->leg_status[LR].prismatic_actuation_force[0]=this->feedback_model_data_mapped_filtered[6].force;
    this->leg_status[LR].prismatic_actuation_force[1]=this->feedback_model_data_mapped_filtered[7].force;
    this->leg_status[LR].prismatic_actuation_force[2]=this->feedback_model_data_mapped_filtered[8].force;

    this->leg_status[RF].prismatic_actuation_force[0]=this->feedback_model_data_mapped_filtered[9].force;
    this->leg_status[RF].prismatic_actuation_force[1]=this->feedback_model_data_mapped_filtered[10].force;
    this->leg_status[RF].prismatic_actuation_force[2]=this->feedback_model_data_mapped_filtered[11].force;

    this->leg_status[RM].prismatic_actuation_force[0]=this->feedback_model_data_mapped_filtered[12].force;
    this->leg_status[RM].prismatic_actuation_force[1]=this->feedback_model_data_mapped_filtered[13].force;
    this->leg_status[RM].prismatic_actuation_force[2]=this->feedback_model_data_mapped_filtered[14].force;

    this->leg_status[RR].prismatic_actuation_force[0]=this->feedback_model_data_mapped_filtered[15].force;
    this->leg_status[RR].prismatic_actuation_force[1]=this->feedback_model_data_mapped_filtered[16].force;
    this->leg_status[RR].prismatic_actuation_force[2]=this->feedback_model_data_mapped_filtered[17].force;

    this->leg_status[LF].prismatic_position[0]=this->feedback_model_data_mapped_filtered[0].position;
    this->leg_status[LF].prismatic_position[1]=this->feedback_model_data_mapped_filtered[1].position;
    this->leg_status[LF].prismatic_position[2]=this->feedback_model_data_mapped_filtered[2].position;

    this->leg_status[LM].prismatic_position[0]=this->feedback_model_data_mapped_filtered[3].position;
    this->leg_status[LM].prismatic_position[1]=this->feedback_model_data_mapped_filtered[4].position;
    this->leg_status[LM].prismatic_position[2]=this->feedback_model_data_mapped_filtered[5].position;

    this->leg_status[LR].prismatic_position[0]=this->feedback_model_data_mapped_filtered[6].position;
    this->leg_status[LR].prismatic_position[1]=this->feedback_model_data_mapped_filtered[7].position;
    this->leg_status[LR].prismatic_position[2]=this->feedback_model_data_mapped_filtered[8].position;

    this->leg_status[RF].prismatic_position[0]=this->feedback_model_data_mapped_filtered[9].position;
    this->leg_status[RF].prismatic_position[1]=this->feedback_model_data_mapped_filtered[10].position;
    this->leg_status[RF].prismatic_position[2]=this->feedback_model_data_mapped_filtered[11].position;

    this->leg_status[RM].prismatic_position[0]=this->feedback_model_data_mapped_filtered[12].position;
    this->leg_status[RM].prismatic_position[1]=this->feedback_model_data_mapped_filtered[13].position;
    this->leg_status[RM].prismatic_position[2]=this->feedback_model_data_mapped_filtered[14].position;

    this->leg_status[RR].prismatic_position[0]=this->feedback_model_data_mapped_filtered[15].position;
    this->leg_status[RR].prismatic_position[1]=this->feedback_model_data_mapped_filtered[16].position;
    this->leg_status[RR].prismatic_position[2]=this->feedback_model_data_mapped_filtered[17].position;

    if(data.time==0)
    {
        rt_printf("Origin prismatic position\n");
        for(int i=0;i<18;i++)
        {
            rt_printf("%f\n",this->feedback_model_data_mapped_filtered[i].position);
        }
    }

    double pVel[18];
    double pAcc[18];
    for(int i=0;i<AXIS_NUMBER;i++)
    {
        //        data.absData.Acceleration[i]=this->m_feedbackModelDataMappedFiltered[i].Acceleration;
        //        data.absData.Position[i]=this->m_feedbackModelDataMappedFiltered[i].Postion;
        //        data.absData.Velocity[i]=this->m_feedbackModelDataMappedFiltered[i].Velocity;
        //        data.absData.PrismActualForce[i]=this->m_feedbackModelDataMappedFiltered[i].Force;
        pAcc[i]=this->feedback_model_data_mapped_filtered[i].acceleration;
        pVel[i]=this->feedback_model_data_mapped_filtered[i].velocity;
        temperary_log_data.prism_acceleration[i]=feedback_model_data_mapped_filtered[i].acceleration;
        temperary_log_data.prism_velocity[i]=feedback_model_data_mapped_filtered[i].velocity;
        temperary_log_data.prism_position[i]=feedback_model_data_mapped_filtered[i].position;
        temperary_log_data.prism_actual_force[i]=feedback_model_data_mapped_filtered[i].force;

    }

    double bodyVel[6]={0,0,0,0,0,0};
    double bodyAcc[6]={0,0,0,0,0,0};

    //at present this function is used to calculating tip forces
    //this means the body position can be regerded as zero
    //    double pBodyEp[]={0,0,0,0,0,0};
    double pAllIn[MOTORS_NUM];
    for(int i =0 ; i<MOTORS_NUM;i++)
    {
        pAllIn[i]=this->feedback_model_data_mapped_filtered[i].position;
    }

    robot.SetPin(pAllIn);// Important step
    robot.SetVin(pVel);
    robot.SetAin(pAcc);


    /*
     this is needed for this application and it's also a way to avoid a bug
     UPDATE: in Robots dev branch, this line could be ommitted, its also run without fault
    this->Robot.SetFixFeet("000000");

     try once for test
    static int test_idx=0;
    if(test_idx!=0)
        return;
    test_idx++;

    double pin[18]={0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7};
    this->Robot.SetPin(pin,this->BodyEpG);
    double ain[18]={0.7,0.3,0.0,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7};
    this->Robot.SetAin(ain,bodyAcc);

    this->Robot.FastDyn();
    this->Robot.pLF->GetFin(this->LegState[LF].prismaticDynForce);
    this->Robot.pLM->GetFin(this->LegState[LM].prismaticDynForce);
    this->Robot.pLR->GetFin(this->LegState[LR].prismaticDynForce);
    this->Robot.pRF->GetFin(this->LegState[RF].prismaticDynForce);
    this->Robot.pRM->GetFin(this->LegState[RM].prismaticDynForce);
    this->Robot.pRR->GetFin(this->LegState[RR].prismaticDynForce);

    rt_printf("%f,%f,%f\n"
              ,this->LegState[0].prismaticDynForce[0]
            ,this->LegState[0].prismaticDynForce[1]
            ,this->LegState[0].prismaticDynForce[2]);

    // test end
    */


    int count=0;
    for(int i=0;i<AXIS_NUMBER;i++)
    {
        if(data.isMotorHomed[i])
            count++;
    }

    if(count==AXIS_NUMBER)
    {
        //let every leg walk in the air
        robot.SetFixFeet("000000");
        robot.FastDyn();

        double fin[18];
        robot.GetFin(fin);

        for(int i=0;i<6;i++)
        {
            this->leg_status[i].prismatic_dynamic_force[0]=fin[i*3];
            this->leg_status[i].prismatic_dynamic_force[1]=fin[i*3+1];
            this->leg_status[i].prismatic_dynamic_force[2]=fin[i*3+2];

        }
        for(int i=0;i<18;i++)
        {
            temperary_log_data.prism_dynamic_force[i]=fin[i];
        }


        for(int i=0;i<6;i++)
        {
            for(int j=0;j<3;j++)
            {
                this->leg_status[i].prismatic_external_force[j]=
                        this->leg_status[i].prismatic_actuation_force[j]-
                        this->leg_status[i].prismatic_dynamic_force[j];
            }
        }

    }




    //    this->Robot.pLF->GetFin(this->LegState[LF].);

    robot.pLF->GetPee(this->leg_status[LF].foot_position_ref_coord,robot.body());
    robot.pLM->GetPee(this->leg_status[LM].foot_position_ref_coord,robot.body());
    robot.pLR->GetPee(this->leg_status[LR].foot_position_ref_coord,robot.body());
    robot.pRF->GetPee(this->leg_status[RF].foot_position_ref_coord,robot.body());
    robot.pRM->GetPee(this->leg_status[RM].foot_position_ref_coord,robot.body());
    robot.pRR->GetPee(this->leg_status[RR].foot_position_ref_coord,robot.body());

    /* 2016-03-28
     * When will the this->Robot.ground() change?
     */
    robot.pLF->GetPee(this->leg_status[LF].foot_postion_abs_coord,beginMak);
    robot.pLM->GetPee(this->leg_status[LM].foot_postion_abs_coord,beginMak);
    robot.pLR->GetPee(this->leg_status[LR].foot_postion_abs_coord,beginMak);
    robot.pRF->GetPee(this->leg_status[RF].foot_postion_abs_coord,beginMak);
    robot.pRM->GetPee(this->leg_status[RM].foot_postion_abs_coord,beginMak);
    robot.pRR->GetPee(this->leg_status[RR].foot_postion_abs_coord,beginMak);

    robot.pLF->GetJfd(*this->leg_status[LF].force_jacobian_direct,robot.body());
    robot.pLM->GetJfd(*this->leg_status[LM].force_jacobian_direct,robot.body());
    robot.pLR->GetJfd(*this->leg_status[LR].force_jacobian_direct,robot.body());
    robot.pRF->GetJfd(*this->leg_status[RF].force_jacobian_direct,robot.body());
    robot.pRM->GetJfd(*this->leg_status[RM].force_jacobian_direct,robot.body());
    robot.pRR->GetJfd(*this->leg_status[RR].force_jacobian_direct,robot.body());

    //calculation forces
    for(int i=0;i<6;i++)
    {
        aris::dynamic::s_dgemm(3,1,3,-1.0,
                               *this->leg_status[i].force_jacobian_direct,3,
                               this->leg_status[i].prismatic_actuation_force,1,
                               0,this->leg_status[i].foot_force_ref_coord,1);

        // calculate extern force
        aris::dynamic::s_dgemm(3,1,3,-1.0,
                               *this->leg_status[i].force_jacobian_direct,3,
                               this->leg_status[i].prismatic_external_force,1,
                               0,this->leg_status[i].foot_external_force_ref_coord,1);

/*
        //        data.absData.ExternForce[i][0]=this->LegState[i].footExternForceM[0];
        //        data.absData.ExternForce[i][1]=this->LegState[i].footExternForceM[1];
        //        data.absData.ExternForce[i][2]=this->LegState[i].footExternForceM[2];
        //a simple rotation
        //        Aris::Dynamic::s_dgemm(3,1,3,1.0,
        //                *this->RotL2M[i],3,
        //                this->LegState[i].footForceL,1,
        //                0,this->LegState[i].footForceM,1);
        //now we have enough forces support

        //GaitPartLeg[i].thrY.threshold(LegState[i].footForceM[1]);// 0 1 2 x y z
        //GaitPartLeg[i].thrZ.threshold(LegState[i].footForceM[2]);//
*/

        // changed for fastdyn
        gait_part_leg[i].threshold_y_positive.threshold(leg_status[i].foot_external_force_ref_coord[1]);
        gait_part_leg[i].thrsehold_z_positive.threshold(leg_status[i].foot_external_force_ref_coord[2]);
        gait_part_leg[i].threshold_z_negative.threshold(leg_status[i].foot_external_force_ref_coord[2]);

        //log
        temperary_log_data.foot_tip_extern_force[i][0]=leg_status[i].foot_external_force_ref_coord[0];
        temperary_log_data.foot_tip_extern_force[i][1]=leg_status[i].foot_external_force_ref_coord[1];
        temperary_log_data.foot_tip_extern_force[i][2]=leg_status[i].foot_external_force_ref_coord[2];

    }
    temperary_log_data.count=data.time;

}

void force_gait::GaitRobot::get_target_pee_robot(double *Pee)
{
    Pee[0]=gait_part_leg[LF].next_target_position[0];
    Pee[1]=gait_part_leg[LF].next_target_position[1];
    Pee[2]=gait_part_leg[LF].next_target_position[2];

    Pee[3]=gait_part_leg[LM].next_target_position[0];
    Pee[4]=gait_part_leg[LM].next_target_position[1];
    Pee[5]=gait_part_leg[LM].next_target_position[2];

    Pee[6]=gait_part_leg[LR].next_target_position[0];
    Pee[7]=gait_part_leg[LR].next_target_position[1];
    Pee[8]=gait_part_leg[LR].next_target_position[2];

    Pee[9]=gait_part_leg[RF].next_target_position[0];
    Pee[10]=gait_part_leg[RF].next_target_position[1];
    Pee[11]=gait_part_leg[RF].next_target_position[2];

    Pee[12]=gait_part_leg[RM].next_target_position[0];
    Pee[13]=gait_part_leg[RM].next_target_position[1];
    Pee[14]=gait_part_leg[RM].next_target_position[2];

    Pee[15]=gait_part_leg[RR].next_target_position[0];
    Pee[16]=gait_part_leg[RR].next_target_position[1];
    Pee[17]=gait_part_leg[RR].next_target_position[2];


    //log second run of the following code casues error
    temperary_log_data.foot_target_coordinate_frame_body[LF][0]=gait_part_leg[LF].next_target_position[0];
    temperary_log_data.foot_target_coordinate_frame_body[LF][1]=gait_part_leg[LF].next_target_position[1];
    temperary_log_data.foot_target_coordinate_frame_body[LF][2]=gait_part_leg[LF].next_target_position[2];

    temperary_log_data.foot_target_coordinate_frame_body[LM][0]=gait_part_leg[LM].next_target_position[0];
    temperary_log_data.foot_target_coordinate_frame_body[LM][1]=gait_part_leg[LM].next_target_position[1];
    temperary_log_data.foot_target_coordinate_frame_body[LM][2]=gait_part_leg[LM].next_target_position[2];

    temperary_log_data.foot_target_coordinate_frame_body[LR][0]=gait_part_leg[LR].next_target_position[0];
    temperary_log_data.foot_target_coordinate_frame_body[LR][1]=gait_part_leg[LR].next_target_position[1];
    temperary_log_data.foot_target_coordinate_frame_body[LR][2]=gait_part_leg[LR].next_target_position[2];

    temperary_log_data.foot_target_coordinate_frame_body[RF][0]=gait_part_leg[RF].next_target_position[0];
    temperary_log_data.foot_target_coordinate_frame_body[RF][1]=gait_part_leg[RF].next_target_position[1];
    temperary_log_data.foot_target_coordinate_frame_body[RF][2]=gait_part_leg[RF].next_target_position[2];

    temperary_log_data.foot_target_coordinate_frame_body[RM][0]=gait_part_leg[RM].next_target_position[0];
    temperary_log_data.foot_target_coordinate_frame_body[RM][1]=gait_part_leg[RM].next_target_position[1];
    temperary_log_data.foot_target_coordinate_frame_body[RM][2]=gait_part_leg[RM].next_target_position[2];

    temperary_log_data.foot_target_coordinate_frame_body[RR][0]=gait_part_leg[RR].next_target_position[0];
    temperary_log_data.foot_target_coordinate_frame_body[RR][1]=gait_part_leg[RR].next_target_position[1];
    temperary_log_data.foot_target_coordinate_frame_body[RR][2]=gait_part_leg[RR].next_target_position[2];

}


// core of this class
void force_gait::GaitRobot::run_gait_robot(
        Robots::RobotBase& rbt,
        ERofoGait cmd,
        Aris::RT_CONTROL::CMachineData &data,
        const Robots::WalkParam &param)
{
    static int BodyMotionLength=2000;
    static long long int BodyMotionStartTime;
    static int BodyMotionCurrentTime=0;



    auto &robot = static_cast<Robots::RobotTypeI &>(rbt);
    //beginMak on the ground,but don't know where it is
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };




    if(param.count==0)
    {
        rt_printf("update beginMak.\n");
        // set postion of beginMak to the current robot postion
        // .pm() always return coordinates of the ground frame
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();

        wait_time=500;
    }

    // robot body with respect to the beginMak
    model_evaluation(robot,beginMak,data);

    robotStateMachine.m_NextState=this->next_motion;
    this->next_motion=this->robot_motion_maker(cmd);

    //    if(data.time%2000==0)
    //    {
    //        rt_printf("ALL State transfer to %s from %s.\n",
    //              ERGS_NAMES[nextMotion-EROFOGAITSTA_OFFSET],
    //            ERGS_NAMES[currentMotion-EROFOGAITSTA_OFFSET]);
    //    }

    switch (this->next_motion)
    {
    case ERS_NULL:
    {
        rt_printf("ERS_NULL\n");
        break;
    }
    case ERS_RNST:
    {
        data=this->standstill_machine_data;
        break;
    }
    case ERS_GFWD:
    {
        // we need update standstiildata in case now it is able to switch to this state at any time
        this->set_standstill_data(data);


        if(this->current_motion!=ERS_GFWD)
        {
            this->obstacle_gait_status_next=OG_INIT;
            this->obstacle_gait_status_current=OG_INIT;

            // start write log data while the robot is going forward
            // data.IsLogging=true;
        }
        switch(obstacle_gait_status_next)
        {
        case OG_INIT:
        {
            //rbt.ground()  robot's cooridnates in the ground cooridnate frame
            //rbt.body()    robot's cooridnates in the body coordinate frame

            beginMak.setPrtPm(*robot.body().pm());
            beginMak.update();

#ifdef IMU
            param.imu_data->toEulBody2Ground(imuEul,PI);
            bodyPeCurrent[3]=imuEul[0];
            bodyPeCurrent[4]=imuEul[1];
            bodyPeCurrent[5]=imuEul[2];
#endif
            //Maybe this line will fix the incontinous problem
            robot.GetPeb(this->bodyPeCurrent,beginMak);

            for(int i=0;i<6;i++)
            {
                BodyPeG[i]=this->bodyPeCurrent[i];
            }

            rt_printf("IMU DATA %f %f %f\n",imu_euler_angles[0],imu_euler_angles[1],imu_euler_angles[2]);


            rt_printf("BodyPeG:%f,%f,%f,%f,%f,%f\n",
                      BodyPeG[0],BodyPeG[1],BodyPeG[2],
                    BodyPeG[3],BodyPeG[4],BodyPeG[5]);
            rt_printf("time: %lld\n",data.time);
            rt_printf("first feedback position: %d\n",data.feedbackData[0].Position);



            this->start_time=data.time;
            this->current_motion=ERS_GFWD;


            // [currentGaitInfo] should not be directly set.
            // it is modified by FirstInitGait
            //
            // [nextGaitInfo] is the candidate of currentGaitInfo
            // and it is referenced in FirstInitGait
            //
            // [nextGaitInfoPos] and [nextGaitInfoForce] is referenced in GaitPart.RunGait
            //
            this->gait_part_leg[LF].next_gait_info_by_position=
                    this->gait_part_leg[LF].gl_MAJOR_FORWARD;
            this->gait_part_leg[LR].next_gait_info_by_position=
                    this->gait_part_leg[LR].gl_MAJOR_FORWARD;
            this->gait_part_leg[RM].next_gait_info_by_position=
                    this->gait_part_leg[RM].gl_MAJOR_FORWARD;

            //used in this gait
            this->gait_part_leg[LF].next_gait_info_by_force=
                    this->gait_part_leg[LF].gl_MINOR_UP_BACKWARD;
            this->gait_part_leg[LR].next_gait_info_by_force=
                    this->gait_part_leg[LR].gl_MINOR_UP_BACKWARD;
            this->gait_part_leg[RM].next_gait_info_by_force=
                    this->gait_part_leg[RM].gl_MINOR_UP_BACKWARD;

            this->gait_part_leg[LF].current_gait_info=
                    this->gait_part_leg[LF].gl_MAJOR_UP;
            this->gait_part_leg[LR].current_gait_info=
                    this->gait_part_leg[LR].gl_MAJOR_UP;
            this->gait_part_leg[RM].current_gait_info=
                    this->gait_part_leg[RM].gl_MAJOR_UP;

            this->gait_part_leg[LF].next_gait_info=
                    this->gait_part_leg[LF].gl_MAJOR_UP;
            this->gait_part_leg[LR].next_gait_info=
                    this->gait_part_leg[LR].gl_MAJOR_UP;
            this->gait_part_leg[RM].next_gait_info=
                    this->gait_part_leg[RM].gl_MAJOR_UP;
            //MAJOR_UP need some modifactions
            // this is needed becasue every cycle init state is irregular
            // the progress is according to the current height and speed
            // decide the destiny pos and totalsteps

            this->gait_part_leg[LM].current_gait_info=
                    this->gait_part_leg[LM].gl_STANDSTILL;
            this->gait_part_leg[RF].current_gait_info=
                    this->gait_part_leg[RF].gl_STANDSTILL;
            this->gait_part_leg[RR].current_gait_info=
                    this->gait_part_leg[RR].gl_STANDSTILL;
            this->gait_part_leg[LM].next_gait_info=
                    this->gait_part_leg[LM].gl_STANDSTILL;
            this->gait_part_leg[RF].next_gait_info=
                    this->gait_part_leg[RF].gl_STANDSTILL;
            this->gait_part_leg[RR].next_gait_info=
                    this->gait_part_leg[RR].gl_STANDSTILL;

            //init actions
            for(int i=0;i<6;i++)
            {
                gait_part_leg[i].is_active=false;
                init_gait(gait_part_leg[i],gait_part_leg[i].next_gait_info,data.time);
            }


            bEE[0]=bodyPeLast[0];
            bEE[1]=bodyPeLast[1];
            bEE[2]=bodyPeLast[2];
            bEE[3]=bodyPeLast[3];
            bEE[4]=bodyPeLast[4];
            bEE[5]=bodyPeLast[5];

            this->gait_part_body.current_gait_info=this->gait_part_body.gb_STANDSTILL;
            gait_part_body.set_original_position(BodyPeG);
            gait_part_body.set_destiny_position(gait_part_body.current_gait_info.delta_position);
            gait_part_body.set_start_time(data.time);

            //normal run


        }
            //break;
        case OG_FIRSTMOTION:

        {
            //transition from OG_INIT
            if(obstacle_gait_status_current!=OG_FIRSTMOTION)
            {
                obstacle_gait_status_next=OG_FIRSTMOTION;
                obstacle_gait_status_current=OG_FIRSTMOTION;
            }
            //            auto imudata=imu.getSensorData();
            //            imudata.get().toEulBody2Ground(imuEul,PI);

#ifdef IMU
            param.imu_data->toEulBody2Ground(imuEul,PI);
            bodyPeCurrent[3]=imuEul[0];
            bodyPeCurrent[4]=imuEul[1];
            bodyPeCurrent[5]=imuEul[2];
#endif

            for(int i=0;i<6;i++)
            {
                BodyPeG[i]=this->bodyPeCurrent[i];
            }

            // above is ok

            //generate next leg point
            this->gait_part_leg[LF].get_next_target_position(this->gait_part_leg[LF].next_target_position,data.time,this->leg_status[LF]);
            this->gait_part_leg[LM].get_next_target_position(this->gait_part_leg[LM].next_target_position,data.time,this->leg_status[LM]);
            this->gait_part_leg[LR].get_next_target_position(this->gait_part_leg[LR].next_target_position,data.time,this->leg_status[LR]);
            this->gait_part_leg[RF].get_next_target_position(this->gait_part_leg[RF].next_target_position,data.time,this->leg_status[RF]);
            this->gait_part_leg[RM].get_next_target_position(this->gait_part_leg[RM].next_target_position,data.time,this->leg_status[RM]);
            this->gait_part_leg[RR].get_next_target_position(this->gait_part_leg[RR].next_target_position,data.time,this->leg_status[RR]);


            // above is ok

            debug_print_gait_part_state(&data.time);

            //detect and switching gait
            for(int i=0;i<6;i++)
            {
                //IsSwitching and m_NextGaitInfo is set by legGait[i].RunGait(); TBD!!!!!!!!!!!!!!!!!!!!!!!!!
                if(gait_part_leg[i].is_need_switching)
                {
                    init_gait(gait_part_leg[i],gait_part_leg[i].next_gait_info,data.time);

                }
            }





            //generate next body point
            gait_part_body.get_next_target_position(gait_part_body.next_target_position,data.time,this->body_status);


            //above is ok

            //calculate next pIN
            for(int i=0;i<6;i++)
            {
                bodyPeCurrent[i]=gait_part_body.next_target_position[i];
                BodyPeG[i]=gait_part_body.next_target_position[i];
                bodyPeLast[i]=gait_part_body.next_target_position[i];

            }


            get_target_pee_robot(pEE);

            robot.SetPeb(gait_part_body.next_target_position,beginMak);

            robot.SetPee(pEE,beginMak);



            if(data.time==start_time)
            {
                debug_print_ee_actual_target(&data.time);
            }
            if(data.time==(start_time+1000))
            {
                debug_print_ee_actual_target(&data.time);
            }

            //            Robot.SetPee(pEE,GaitPartBody.targetEndPos,"G");
            //            Robot.GetPin(pIN);
            //            for(int i=0;i<18;i++)
            //            {
            //                m_commandModelDataMapped[i].Postion=pIN[i];
            //            }

            //            PostProcess(data);

            //            //for safe
            //            this->Safety(data);


            // end jump section for test only
            int Count;
            Count=0;
            for(int i=0;i<6;i++)
            {
                if(this->gait_part_leg[i].current_gait_info.gait==GAIT_STANDSTILL)
                {
                    Count++;
                }
            }
            if(Count==6)
            {
                obstacle_gait_status_next=OG_BODYMOTION;
                rt_printf("Change to body motion %f,%f,%f",pEE[0],pEE[1],pEE[2]);
            }

            //above is the error, let's hunt it down.



            break;
        }
        case OG_BODYMOTION:
        {
            if(obstacle_gait_status_current!=OG_BODYMOTION)
            {
                //                auto imudata=imu.getSensorData();
                //                imudata.get().toEulBody2Ground(imuEul,PI);
#ifdef IMU
                param.imu_data->toEulBody2Ground(imuEul,PI);
                bodyPeCurrent[3]=imuEul[0];
                bodyPeCurrent[4]=imuEul[1];
                bodyPeCurrent[5]=imuEul[2];
#endif
                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=this->bodyPeCurrent[i];
                };

                rt_printf("body_motion begin\n");
                obstacle_gait_status_current=OG_BODYMOTION;
                //first cycle
                //in this stage pEE will use previous one,do not change it

                //                GaitPartBody.currentGaitInfo.deltaPos[2]=
                //                        LegState[LF].footPosM[2]-
                //                        GaitPartLeg[LF].obstacleRefPosM[2];


                gait_part_body.current_gait_info=gait_part_body.gb_MAJOR_FORWARD;
                rt_printf("Body deltaPos z:%f\n",gait_part_body.current_gait_info.delta_position[2]);
                gait_part_body.current_gait_info.delta_position[2]=
                        leg_status[LF].foot_position_ref_coord[2]-
                        gait_part_leg[LF].gait_ref_position_body_coord[2];
                // deltaPos[4] is the 1 in 313
                // change it here and the BodyEpG will be assigned with this value
//                gait_part_body.current_gait_info.delta_position[4]=-imu_euler_angles[2];
                //for 805
                //                GaitPartBody.currentGaitInfo.deltaPos[2]=-0.2;
                rt_printf("Body deltaPos z:%f\n",gait_part_body.current_gait_info.delta_position[2]);





                rt_printf("IMU DATA %f %f %f\n",imu_euler_angles[0],imu_euler_angles[1],imu_euler_angles[2]);

                init_gait(gait_part_body,gait_part_body.current_gait_info,data.time);


                bEE[0]=bodyPeLast[0];
                bEE[1]=bodyPeLast[1];
                bEE[2]=bodyPeLast[2];
                bEE[3]=bodyPeLast[3];
                bEE[4]=bodyPeLast[4];
                bEE[5]=bodyPeLast[5];


                //                rt_printf("body origin:%f,%f,%f,%f,%f,%f\n",
                //                          bEE[0],bEE[1],bEE[2],
                //                        bEE[3],bEE[4],bEE[5]);
                //                GaitPartBody.totalSteps=GaitPartBody.currentGaitInfo.totalSteps;
                //                GaitPartBody.SetOriginPos(bodyEpLast);
                //                GaitPartBody.IsNeedSwitching=false;
                //                GaitPartBody.totalSteps=GaitPartBody.currentGaitInfo.deltaPos[5]/
                //                        GaitPartBody.currentGaitInfo.speed[5];
                rt_printf("Body Motion Steps %d\n",gait_part_body.total_steps);
                //                GaitPartBody.totalSteps=4000;
                //                GaitPartBody.SetDestinyPos(GaitPartBody.currentGaitInfo.deltaPos);
                //                rt_printf("body origin:%f,%f,%f,%f,%f,%f\n",
                //                        GaitPartBody.originEndPos[0],
                //                        GaitPartBody.originEndPos[1],
                //                        GaitPartBody.originEndPos[2],
                //                        GaitPartBody.originEndPos[3],
                //                        GaitPartBody.originEndPos[4],
                //                        GaitPartBody.originEndPos[5]);
                //                rt_printf("body dest:%f,%f,%f,%f,%f,%f\n",
                //                        GaitPartBody.destEndPos[0],
                //                        GaitPartBody.destEndPos[1],
                //                        GaitPartBody.destEndPos[2],
                //                        GaitPartBody.destEndPos[3],
                //                        GaitPartBody.destEndPos[4],
                //                        GaitPartBody.destEndPos[5]);
                //                GaitPartBody.SetStartTime(data.time);

                //normal
                rt_printf("before GetAllPee %f,%f,%f\n",pEE[0],pEE[1],pEE[2]);
                get_target_pee_robot(pEE);
                rt_printf("after GetAllPee %f,%f,%f\n",pEE[0],pEE[1],pEE[2]);

                gait_part_body.get_next_target_position(gait_part_body.next_target_position,data.time,this->body_status);
                rt_printf("body:%f,%f,%f,%f,%f,%f\n",bEE[0],bEE[1],bEE[2],
                        bEE[3],bEE[4],bEE[5]);
                // modify here for x-sin(x) change target end pos

                BodyMotionStartTime=data.time;
                BodyMotionCurrentTime=(int)(data.time-BodyMotionStartTime);
                double bmtime
                        =((double)BodyMotionCurrentTime)
                        /((double)BodyMotionLength)
                        *2.0*3.1415926;
                gait_part_body.next_target_position[2]
                        =gait_part_body.current_gait_info.delta_position[2]
                        *1.0/(2.0*3.1415926)*(bmtime-sin(bmtime))
                        +gait_part_body.original_position[2];

//                gait_part_body.next_target_position[4]
//                        =gait_part_body.current_gait_info.delta_position[4]
//                        *1.0/(2.0*3.1415926)*(bmtime-sin(bmtime))
//                        +gait_part_body.original_position[4];


                //calculate next pIN
                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=gait_part_body.next_target_position[i];
                    bodyPeCurrent[i]=gait_part_body.next_target_position[i];
                    bodyPeLast[i]=gait_part_body.next_target_position[i];
                }



                robot.SetPeb(gait_part_body.next_target_position,beginMak);
                robot.SetPee(pEE,beginMak);

                //                Robot.SetPee(pEE,GaitPartBody.targetEndPos,"G");
                if(gait_part_body.current_step<10)
                {
                    rt_printf("pEE:%f\n",pEE[0]);
                }
                //                Robot.GetPin(pIN);
                //                for(int i=0;i<18;i++)
                //                {
                //                    m_commandModelDataMapped[i].Postion=pIN[i];
                //                }

                //                PostProcess(data);

                //                //for safe
                //                this->Safety(data);

                if(gait_part_body.is_need_switching)
                {
                    gait_part_body.current_gait_info=gait_part_body.gb_STANDSTILL;
                    gait_part_body.next_gait_info_by_force=gait_part_body.gb_STANDSTILL;
                    gait_part_body.next_gait_info_by_position=gait_part_body.gb_STANDSTILL;
                    gait_part_body.set_original_position(bodyPeLast);
                }
            }
            else
            {

                //                auto imudata=imu.getSensorData();
                //                imudata.get().toEulBody2Ground(imuEul,PI);
#ifdef IMU
                param.imu_data->toEulBody2Ground(imuEul,PI);
                bodyPeCurrent[3]=imuEul[0];
                bodyPeCurrent[4]=imuEul[1];
                bodyPeCurrent[5]=imuEul[2];
#endif
                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=this->bodyPeCurrent[i];
                }

                //normal
                get_target_pee_robot(pEE); // pEE will not change in this stage
                gait_part_body.get_next_target_position(gait_part_body.next_target_position,data.time,this->body_status);

                BodyMotionCurrentTime=(int)(data.time-BodyMotionStartTime);
                double bmtime
                        =((double)BodyMotionCurrentTime)
                        /((double)BodyMotionLength)
                        *2.0*3.1415926;
                gait_part_body.next_target_position[2]
                        =gait_part_body.current_gait_info.delta_position[2]
                        *1.0/(2.0*3.1415926)*(bmtime-sin(bmtime))
                        +gait_part_body.original_position[2];

//                gait_part_body.next_target_position[4]
//                        =gait_part_body.current_gait_info.delta_position[4]
//                        *1.0/(2.0*3.1415926)*(bmtime-sin(bmtime))
//                        +gait_part_body.original_position[4];
                //calculate next pIN

                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=gait_part_body.next_target_position[i];
                    bodyPeCurrent[i]=gait_part_body.next_target_position[i];
                    bodyPeLast[i]=gait_part_body.next_target_position[i];
                }
                robot.SetPeb(gait_part_body.next_target_position,beginMak);
                robot.SetPee(pEE,beginMak);
                //                Robot.SetPee(pEE,GaitPartBody.targetEndPos,"G");

                //                Robot.GetPin(pIN);
                //                for(int i=0;i<18;i++)
                //                {
                //                    m_commandModelDataMapped[i].Postion=pIN[i];
                //                }

                //                PostProcess(data);

                //                //for safe
                //                this->Safety(data);



                //                if(GaitPartBody.IsNeedSwitching)
                //                {
                //                    GaitPartBody.currentGaitInfo=GaitPartBody.gb_STANDSTILL;
                //                    GaitPartBody.nextGaitInfoForce=GaitPartBody.gb_STANDSTILL;
                //                    GaitPartBody.nextGaitInfoPos=GaitPartBody.gb_STANDSTILL;
                //                    GaitPartBody.SetOriginPos(bodyPeLast);

                //                    obstacleGaitStateNext=OG_SECONDMOTION;
                //                    GaitPartBody.IsNeedSwitching=false;
                //                }
                if(BodyMotionCurrentTime>=BodyMotionLength)
                {
                    gait_part_body.current_gait_info=gait_part_body.gb_STANDSTILL;
                    gait_part_body.next_gait_info_by_force=gait_part_body.gb_STANDSTILL;
                    gait_part_body.next_gait_info_by_position=gait_part_body.gb_STANDSTILL;
                    gait_part_body.set_original_position(bodyPeLast);

                    obstacle_gait_status_next=OG_SECONDMOTION;
                    gait_part_body.is_need_switching=false;

                }
            }
            break;
        }
        case OG_SECONDMOTION:
        {
            if(obstacle_gait_status_current!=OG_SECONDMOTION)
            {
                rt_printf("Second Motion begin\n");
#ifdef IMU
                param.imu_data->toEulBody2Ground(imuEul,PI);
                bodyPeCurrent[3]=imuEul[0];
                bodyPeCurrent[4]=imuEul[1];
                bodyPeCurrent[5]=imuEul[2];
#endif
                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=this->bodyPeCurrent[i];
                }


                rt_printf("IMU DATA %f %f %f\n",imu_euler_angles[0],imu_euler_angles[1],imu_euler_angles[2]);

                obstacle_gait_status_current=OG_SECONDMOTION;

                this->gait_part_leg[RF].next_gait_info_by_position=
                        this->gait_part_leg[RF].gl_MAJOR_FORWARD;
                this->gait_part_leg[RR].next_gait_info_by_position=
                        this->gait_part_leg[RR].gl_MAJOR_FORWARD;
                this->gait_part_leg[LM].next_gait_info_by_position=
                        this->gait_part_leg[LM].gl_MAJOR_FORWARD;



                //not used in this gait
                this->gait_part_leg[RF].next_gait_info_by_force=
                        this->gait_part_leg[RF].gl_MINOR_UP_BACKWARD;
                this->gait_part_leg[RR].next_gait_info_by_force=
                        this->gait_part_leg[RR].gl_MINOR_UP_BACKWARD;
                this->gait_part_leg[LM].next_gait_info_by_force=
                        this->gait_part_leg[LM].gl_MINOR_UP_BACKWARD;

                this->gait_part_leg[RF].current_gait_info=
                        this->gait_part_leg[RF].gl_MAJOR_UP;
                this->gait_part_leg[RR].current_gait_info=
                        this->gait_part_leg[RR].gl_MAJOR_UP;
                this->gait_part_leg[LM].current_gait_info=
                        this->gait_part_leg[LM].gl_MAJOR_UP;

                this->gait_part_leg[RF].next_gait_info=
                        this->gait_part_leg[RF].gl_MAJOR_UP;
                this->gait_part_leg[RR].next_gait_info=
                        this->gait_part_leg[RR].gl_MAJOR_UP;
                this->gait_part_leg[LM].next_gait_info=
                        this->gait_part_leg[LM].gl_MAJOR_UP;
                //MAJOR_UP need some modifactions
                // this is needed becasue every cycle init state is irregular
                // the progress is according to the current height and speed
                // decide the destiny pos and totalsteps

                this->gait_part_leg[RM].current_gait_info=
                        this->gait_part_leg[RM].gl_STANDSTILL;
                this->gait_part_leg[LF].current_gait_info=
                        this->gait_part_leg[LF].gl_STANDSTILL;
                this->gait_part_leg[LR].current_gait_info=
                        this->gait_part_leg[LR].gl_STANDSTILL;

                //init actions
                for(int i=0;i<6;i++)
                {
                    init_gait(gait_part_leg[i],gait_part_leg[i].next_gait_info,data.time);
                }

                this->gait_part_body.current_gait_info=this->gait_part_body.gb_STANDSTILL;

                gait_part_body.set_original_position(bodyPeLast);
                gait_part_body.set_destiny_position(gait_part_body.current_gait_info.delta_position);
                gait_part_body.set_start_time(data.time);

            }
            //            else
            {
#ifdef IMU
                param.imu_data->toEulBody2Ground(imuEul,PI);
                bodyPeCurrent[3]=imuEul[0];
                bodyPeCurrent[4]=imuEul[1];
                bodyPeCurrent[5]=imuEul[2];
#endif
                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=this->bodyPeCurrent[i];
                }

                //generate next leg point
                this->gait_part_leg[LF].get_next_target_position(this->gait_part_leg[LF].next_target_position,data.time,this->leg_status[LF]);
                this->gait_part_leg[LM].get_next_target_position(this->gait_part_leg[LM].next_target_position,data.time,this->leg_status[LM]);
                this->gait_part_leg[LR].get_next_target_position(this->gait_part_leg[LR].next_target_position,data.time,this->leg_status[LR]);
                this->gait_part_leg[RF].get_next_target_position(this->gait_part_leg[RF].next_target_position,data.time,this->leg_status[RF]);
                this->gait_part_leg[RM].get_next_target_position(this->gait_part_leg[RM].next_target_position,data.time,this->leg_status[RM]);
                this->gait_part_leg[RR].get_next_target_position(this->gait_part_leg[RR].next_target_position,data.time,this->leg_status[RR]);



                //detect and switching gait
                for(int i=0;i<6;i++)
                {
                    //IsSwitching and m_NextGaitInfo is set by legGait[i].RunGait(); TBD!!!!!!!!!!!!!!!!!!!!!!!!!
                    if(gait_part_leg[i].is_need_switching)
                    {
                        init_gait(gait_part_leg[i],gait_part_leg[i].next_gait_info,data.time);

                    }
                }


                //generate next body point
                gait_part_body.get_next_target_position(gait_part_body.next_target_position,data.time,this->body_status);

                //calculate next pIN
                for(int i=0;i<6;i++)
                {
                    BodyPeG[i]=gait_part_body.next_target_position[i];
                    bodyPeCurrent[i]=gait_part_body.next_target_position[i];
                    bodyPeLast[i]=gait_part_body.next_target_position[i];
                }

                get_target_pee_robot(pEE);

                robot.SetPeb(gait_part_body.next_target_position,beginMak);
                robot.SetPee(pEE,beginMak);

                //                Robot.SetPee(pEE,GaitPartBody.targetEndPos,"G");
                //                Robot.GetPin(pIN);
                //                for(int i=0;i<18;i++)
                //                {
                //                    m_commandModelDataMapped[i].Postion=pIN[i];

                //                }

                //                PostProcess(data);

                //                //for safe
                //                this->Safety(data);



                // end jump section for test only
                int Count;
                Count=0;
                for(int i=0;i<6;i++)
                {
                    if(this->gait_part_leg[i].current_gait_info.gait==GAIT_STANDSTILL)
                    {
                        Count++;
                    }
                }
                if(Count==6)
                {
                    this->set_standstill_data(data);
                    data=this->standstill_machine_data;
//                    rt_printf("CLIMB FINISHED\n");
                    // this is a symbol that we can use outside the object
                    wait_time--;
                    if(wait_time<0)
                    {
                        this->next_motion=ERS_RNST;
                        //here we do some reset works
                        for(int i=0;i<6;i++)
                        {
                            gait_part_leg[i].is_active=false;
                            gait_part_leg[i].is_need_switching=false;
                        }
                        robot.GetPin(pIN);
                        rt_printf("Last position\n");
                        for(int i=0;i<18;i++)
                        {
                            rt_printf("%f\n",pIN[i]);
                        }
                        rt_printf("CLIMB FINISHED\n");
                    }

                }


            }

            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        rt_printf("Default %d\n",next_motion);
        this->set_standstill_data(data);
        data=this->standstill_machine_data;
        this->next_motion=ERS_RNST;
        break;
    }

    for(int i=0;i<6;i++)
    {
        temperary_log_data.body_center_coordinate_frame_ground[i]=gait_part_body.next_target_position[i];
    }

    int err=0;
    err=rt_dev_sendto(robot_log.file_id_real_time
                      ,&temperary_log_data
                      ,sizeof(temperary_log_data)
                      ,0,NULL,0);

}


// for 3d motion
void force_gait::GaitRobot::init_gait(GaitPart<3> &gp, GaitInformation<3> nextGaitInfo, long long time)
{
    switch(nextGaitInfo.gait)
    {
    case GAIT_STRAIGHT:
    {

        rt_printf("GAIT_STRAIGHT init %s.\n",LEG_NAME[gp.id]);
        gp.current_gait_info=nextGaitInfo;

        gp.set_original_position(leg_status[gp.id].foot_postion_abs_coord);
        //FirstInitGait is used to determine DeltaEndPos
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);
        gp.current_gait_info.max_acceleration=0.1;
        gp.current_gait_info.max_deceleration=0.1;
        gp.current_gait_info.max_velocity=0.1;
        gp.is_need_switching=false;
        break;
    }
    case GAIT_MAJOR_UP:
        rt_printf("GAIT_MAJOR_UP init %s.\n",LEG_NAME[gp.id]);
        gp.current_gait_info=nextGaitInfo;
        gp.current_gait_info.gait=GAIT_MAJOR_UP;
        //adjust DestinyEndPos according to M coordinates
        //x z cooridinate is ok
        gp.current_gait_info.delta_position[0]=0;
        gp.current_gait_info.delta_position[2]=0;
        //adjust Y to a proper position relative to m_ObstacleRefPosM
        if(leg_status[gp.id].foot_position_ref_coord[1]<(gp.gait_ref_position_body_coord[1]+nextGaitInfo.delta_position[1]))
        {
            rt_printf("1 %f %f %f\n",leg_status[gp.id].foot_position_ref_coord[1],
                    nextGaitInfo.delta_position[1],
                    gp.position_limit[1][0]);
            gp.current_gait_info.delta_position[1]=
                    gp.gait_ref_position_body_coord[1]+nextGaitInfo.delta_position[1]-leg_status[gp.id].foot_position_ref_coord[1];
        }
        else if((leg_status[gp.id].foot_position_ref_coord[1]+nextGaitInfo.delta_position[1])<gp.position_limit[1][0])
        {
            rt_printf("2 %f %f %f\n",leg_status[gp.id].foot_position_ref_coord[1],
                    nextGaitInfo.delta_position[1],
                    gp.position_limit[1][0]);
            gp.current_gait_info.delta_position[1]=nextGaitInfo.delta_position[1];
        }
        else
        {
            rt_printf("Limit touched %f\n",leg_status[gp.id].foot_position_ref_coord[1]);

            // here may need some offset or may cause error
            gp.current_gait_info.delta_position[1]=
                    gp.position_limit[1][0]-leg_status[gp.id].foot_position_ref_coord[1];

        }
        rt_printf("GAIT_MAJOR_UP delta Y pos: %f \n",gp.current_gait_info.delta_position[1]);
        //        gp.currentGaitInfo.deltaPos[1]=0.05;
        //calculate total steps
        gp.current_gait_info.total_steps=
                (int)(gp.current_gait_info.delta_position[1]/gp.current_gait_info.velocity[1]);

        //2016-04-13 this one is different, it the very first one,
        // so there is no precedent targetEndPos here.
        //        gp.SetOriginPos(LegState[gp.ID].footPosG);
        //        gp.SetOriginPos(gp.targetEndPos);
        if(gp.is_active)
        {
            gp.set_original_position(gp.destiny_position);
        }
        else
        {
            gp.set_original_position(leg_status[gp.id].foot_postion_abs_coord);
            gp.is_active=true;

        }
        //FirstInitGait is used to determine DeltaEndPos
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);
        gp.current_gait_info.max_acceleration=0.1;
        gp.current_gait_info.max_deceleration=0.05;
        gp.current_gait_info.max_velocity=0.1;
        gp.is_need_switching=false;
        break;
    case GAIT_MAJOR_FORWARD: // move alone z
        rt_printf("GAIT_MAJOR_FORWARD init %s.\n",LEG_NAME[gp.id]);
        if(gp.id%2==0)
        {
            // new threshold for fastdyn
            gp.thrsehold_z_positive.set_threshold(gp.limit_z_positive_lo,gp.limit_z_positive_hi);
            gp.thrsehold_z_positive.reset();
            gp.current_gait_info=nextGaitInfo;
            //adjust DestinyEndPos according to M coordinates
            //x y cooridinate is ok
            gp.current_gait_info.delta_position[0]=0;
            gp.current_gait_info.delta_position[1]=0;
            //adjust Z to a proper position relative to m_ObstacleRefPosM

            // this z offset may include

            gp.current_gait_info.delta_position[2]=
                    gp.gait_ref_position_body_coord[2]+
                    gp.gl_MAJOR_FORWARD.delta_position[2]-
                    leg_status[gp.id].foot_position_ref_coord[2];
            rt_printf("DELTA POS Z %s %f\n",
                      LEG_NAME[gp.id],
                      gp.current_gait_info.delta_position[2]);


            //  this value is determined by MINOR_UP_BACKWARD
            //  0.03(backward displacement) + 0.05(foot radius)
            if(gp.current_gait_info.delta_position[2]>-0.10)
            {
                gp.current_gait_info.delta_position[2]=-0.10;

            }

            rt_printf("DELTA POS Z %s %f\n",
                      LEG_NAME[gp.id],
                      gp.current_gait_info.delta_position[2]);




            gp.current_gait_info.total_steps=
                    (int)(gp.current_gait_info.delta_position[2]/gp.current_gait_info.velocity[2]);

        }
        else
        {
            // new threshold for fastdyn
            gp.thrsehold_z_positive.set_threshold(gp.limit_z_positive_lo,gp.limit_z_positive_hi);
            gp.thrsehold_z_positive.reset();
            gp.current_gait_info=nextGaitInfo;
            //adjust DestinyEndPos according to M coordinates
            //x y cooridinate is ok
            gp.current_gait_info.delta_position[0]=0;
            gp.current_gait_info.delta_position[1]=0;
            //adjust Z to a proper position relative to m_ObstacleRefPosM

            gp.current_gait_info.delta_position[2]=
                    gp.gait_ref_position_body_coord[2]-
                    leg_status[gp.id].foot_position_ref_coord[2];

            //  this value is determined by MINOR_UP_BACKWARD
            //  0.03(backward displacement) + 0.05(foot radius)
            if(gp.current_gait_info.delta_position[2]>-0.10)
            {
                gp.current_gait_info.delta_position[2]=-0.10;

            }

            rt_printf("DELTA POS Z %s %f\n",
                      LEG_NAME[gp.id],
                      gp.current_gait_info.delta_position[2]);

            gp.current_gait_info.total_steps=
                    (int)(gp.current_gait_info.delta_position[2]/gp.current_gait_info.velocity[2]);

        }


        //        gp.SetOriginPos(LegState[gp.ID].footPosG);
        //this is the last target position
        gp.set_original_position(gp.next_target_position);
        //FirstInitGait is used to determine DeltaEndPos
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);
        gp.is_need_switching=false;
        break;
    case GAIT_MAJOR_DOWN:
        rt_printf("GAIT_MAJOR_DOWN init %s.\n",LEG_NAME[gp.id]);
        gp.threshold_y_positive.set_threshold(gp.limit_y_positive_lo,gp.limit_y_positive_hi);
        gp.threshold_y_positive.reset();
        gp.current_gait_info=nextGaitInfo;
        gp.current_gait_info.max_acceleration=0.05;
        gp.current_gait_info.max_deceleration=0.05;
        gp.current_gait_info.max_velocity=0.1;
        //adjust DestinyEndPos according to M coordinates
        //x z cooridinate is ok
        gp.current_gait_info.delta_position[0]=0;
        gp.current_gait_info.delta_position[2]=0;
        //here add an offset -0.05 to make sure force contact
        //if disable force detect, this offset will cause first motion
        // MAJOR_DOWN move the body up.

        gp.current_gait_info.delta_position[1]=
                gp.gait_ref_position_body_coord[1]-0.01-
                leg_status[gp.id].foot_position_ref_coord[1];

        //        gp.SetOriginPos(LegState[gp.ID].footPosG);
        gp.set_original_position(gp.destiny_position);
        //FirstInitGait is used to determine DeltaEndPos
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);
        gp.is_need_switching=false;

        break;

    case GAIT_MINOR_UP_BACKWARD:
        rt_printf("GAIT_MINOR_UP_BACKWARD init %s.\n",LEG_NAME[gp.id]);
        gp.current_gait_info=nextGaitInfo;

        //this is all right;
        //        gp.SetOriginPos(LegState[gp.ID].footPosG);

        //2016-04-21 here we use next_target_position rather than destiny_position
        // in fact, all of them should use next_target_position, or the gait is unbreakable.
        // but
        gp.set_original_position(gp.next_target_position);

        //FirstInitGait is used to determine DeltaEndPos
        //x z is ok
        //check if y is too high
        if((gp.current_gait_info.delta_position[1]+leg_status[gp.id].foot_position_ref_coord[1])
                >gp.position_limit[1][0])
        {
            rt_printf("can't back\n");
            gp.current_gait_info.delta_position[1]=gp.position_limit[1][0]-leg_status[gp.id].foot_position_ref_coord[1];
        }
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);



        gp.is_need_switching=false;
        rt_printf("%f,%f,%f\n",
                  gp.destiny_position[0],
                gp.destiny_position[1],
                gp.destiny_position[2]);
        rt_printf("%f,%f,%f\n",gp.current_gait_info.delta_position[0],
                gp.current_gait_info.delta_position[1],
                gp.current_gait_info.delta_position[2]);
        break;
    case GAIT_STANDSTILL:
        rt_printf("GAIT_STANDSTILL init %s.\n",LEG_NAME[gp.id]);
        gp.current_gait_info=nextGaitInfo;

        if(gp.is_active)
        {
            gp.set_original_position(gp.next_target_position);
        }
        else
        {
            gp.set_original_position(leg_status[gp.id].foot_postion_abs_coord);
            gp.is_active=true;

        }

        //FirstInitGait is used to determine DeltaEndPos
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);
        gp.is_need_switching=false;
        break;
    case GAIT_H2S:
        rt_printf("GAIT_H2S init %s.\n",LEG_NAME[gp.id]);
        gp.current_gait_info=nextGaitInfo;
        gp.set_original_position(leg_status[gp.id].foot_postion_abs_coord);
        //        gp.SetOriginPos(gp.destEndPos);
        //FirstInitGait is used to determine DeltaEndPos
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        gp.set_start_time(time);
        gp.is_need_switching=false;
        break;

    case GAIT_LOCAL_CYCLE_FOREWARD:
        gp.thrsehold_z_positive.set_threshold(100,200);
        gp.thrsehold_z_positive.reset();
        //maybe the following value is too big

        //        gp.thrZpositive.setThr(100,500);
        //        gp.thrZpositive.Reset();
        rt_printf("GAIT_LOCAL_CYCLE_FOREWARD init. %s \n",LEG_NAME[gp.id]);
        // start point of cycle foreward
        gp.current_gait_info=nextGaitInfo;
        // this start param should be OK
        gp.local_smooth_param.Init(-0.001,1.5*M_PI);
        // z-axis of the robot coordinate frame is x in circle
        // y-axis of the robot coordinate frame is y in circle
        // the first value is LegState[gp.ID].footPosG[2]-0.3
        // i think this is a bug, because the center of the circle
        // should not be so faraway from the current point,(30cm, is that possible?)
        // if this is a bug, then the question is why in the practice
        // the trajectory is discontinuous but the gap is not that big.

        gp.local_cycle.SetCycleParam(
                    leg_status[gp.id].foot_postion_abs_coord[2]-0.03,
                leg_status[gp.id].foot_postion_abs_coord[1],
                0.03,0.03,-0.001,false);
        gp.local_cycle.SetSwitchPoint(0.0,'y',M_PI,'y');
        if(gp.id==0||gp.id==3)
        {
            rt_printf("cycle forward origin pos %f %f %f\n",
                      leg_status[gp.id].foot_postion_abs_coord[0],
                    leg_status[gp.id].foot_postion_abs_coord[1],
                    leg_status[gp.id].foot_postion_abs_coord[2]);

        }
        gp.total_steps=5000;

        //        gp.SetOriginPos(LegState[gp.ID].footPosG);
        gp.set_original_position(gp.destiny_position);
        gp.set_start_time(time);
        gp.is_need_switching=false;

        break;
    case GAIT_LOCAL_CYCLE_BACKWARD:
    {

        rt_printf("GAIT_LOCAL_CYCLE_BACKWARD init. %s ",LEG_NAME[gp.id]);
        // start point of cycle backward
        // reverse need more careful
        gp.current_gait_info=nextGaitInfo;
        gp.thrsehold_z_positive.set_threshold(gp.limit_z_positive_lo,gp.limit_z_positive_hi);
        gp.thrsehold_z_positive.reset();

        // this pos[2] is used to calclaute initAngle
        double pos[2]={leg_status[gp.id].foot_postion_abs_coord[2],leg_status[gp.id].foot_postion_abs_coord[1]};


        //this can cause error
        // but  i don't the observed backward offset is causeb by
        // this line, so let's commented it and try again
        //        pos[0]+=gp.localCycle.StartX;
        //        pos[1]+=gp.localCycle.StartY;

        double center[2];
        double radiusA;
        double radiusB;

        if(!gp.threshold_y_positive.is_on())
        {
            rt_printf("a big step.\n");
            // a big backward is needed
            center[0]=gp.local_cycle.CenterX+0.03/2.0;
            center[1]=gp.local_cycle.CenterY;
            radiusA=gp.local_cycle.RadiusA+0.03/2.0;
            radiusB=gp.local_cycle.RadiusB;

            gp.total_steps=7000;
        }
        else
        {
            rt_printf("back to middle position");
            center[0]=gp.local_cycle.CenterX-gp.local_cycle.RadiusA/2.0;
            center[1]=gp.local_cycle.CenterY;
            // back to the middle place
            radiusA=gp.local_cycle.RadiusA/2.0;
            radiusB=gp.local_cycle.RadiusB;
            gp.total_steps=5000;
        }
        pos[0]-=center[0];
        pos[1]-=center[1];
        double initAngle=atan2(pos[1],pos[0]);
        if(gp.local_cycle.IsClockwise)
        {}
        else
        {
            if(initAngle<0.0)
            {
                initAngle+=2*M_PI;
            }
        }


        gp.local_smooth_param.Init(gp.local_cycle.CurrentParam,-0.5*M_PI);
        gp.local_cycle.SetCycleParam(
                    center[0],
                center[1],
                radiusA,
                radiusB,
                gp.local_cycle.CurrentParam,
                true);



        gp.local_cycle.SetSwitchPoint(M_PI,'y',0.0,'y');

        //        gp.SetOriginPos(LegState[gp.ID].footPosG);
        gp.set_original_position(gp.destiny_position);
        gp.set_start_time(time);
        gp.is_need_switching=false;
        break;
    }

    default:
        break;
    }
    //**Smooth the line based on current gait info//

    double absDeltaPos[3];
    double maxDeltaPos=0;
    for(int i=0;i<3;i++)
    {
        // abs delta and find max delta to caclulate ratio
        absDeltaPos[i]=abs(gp.current_gait_info.delta_position[i]);
        if(absDeltaPos[i]>maxDeltaPos)
        {
            maxDeltaPos=absDeltaPos[i];
        }
    }
    if(gp.current_gait_info.gait!=GAIT_STANDSTILL&&
            gp.current_gait_info.gait!=GAIT_LOCAL_CYCLE_BACKWARD&&
            gp.current_gait_info.gait!=GAIT_LOCAL_CYCLE_FOREWARD)
    {
        gp.current_gait_info.total_steps=0;
    }



    if(gp.current_gait_info.gait!=GAIT_STANDSTILL&&
            gp.current_gait_info.gait!=GAIT_LOCAL_CYCLE_BACKWARD&&
            gp.current_gait_info.gait!=GAIT_LOCAL_CYCLE_FOREWARD)
    {
        for(int i=0;i<3;i++)
        {
            // judge direction
            if(gp.current_gait_info.delta_position[i]<0)
            {
                gp.current_gait_info.direction[i]=-1;
            }
            else
            {
                gp.current_gait_info.direction[i]=1;
            }

            gp.current_gait_info.ratio[i]=absDeltaPos[i]/maxDeltaPos;

            gp.current_gait_info.actual_acceleration[i]=gp.current_gait_info.max_acceleration*
                    gp.current_gait_info.ratio[i];
            gp.current_gait_info.actual_deceleration[i]=gp.current_gait_info.max_deceleration*
                    gp.current_gait_info.ratio[i];
            gp.current_gait_info.actual_velocity[i]=gp.current_gait_info.max_velocity*
                    gp.current_gait_info.ratio[i];

            //
            double TaccMAX=gp.current_gait_info.actual_velocity[i]/gp.current_gait_info.max_acceleration;
            double TdecMAX=gp.current_gait_info.actual_velocity[i]/gp.current_gait_info.max_deceleration;
            gp.current_gait_info.time_no_constant_speed[i]=
                    0.5*gp.current_gait_info.actual_acceleration[i]*TaccMAX*TaccMAX+
                    0.5*gp.current_gait_info.actual_deceleration[i]*TdecMAX*TdecMAX;
            if(((double)gp.current_gait_info.direction[i]*gp.current_gait_info.delta_position[i])<
                    gp.current_gait_info.time_no_constant_speed[i])
            {
                gp.current_gait_info.time_acceleration[i]=
                        (sqrt(2.0*gp.current_gait_info.actual_deceleration[i]*absDeltaPos[i]))
                        /(sqrt(gp.current_gait_info.actual_acceleration[i]*gp.current_gait_info.actual_acceleration[i]+gp.current_gait_info.actual_acceleration[i]*gp.current_gait_info.actual_deceleration[i]));

                gp.current_gait_info.time_deceleration[i]=
                        (sqrt(2.0)*gp.current_gait_info.actual_acceleration[i]*sqrt(absDeltaPos[i]))
                        /(sqrt(gp.current_gait_info.actual_deceleration[i]*gp.current_gait_info.actual_acceleration[i]*(gp.current_gait_info.actual_deceleration[i]+gp.current_gait_info.actual_acceleration[i])));
                gp.current_gait_info.time_constant[i]=0;
            }
            else
            {
                gp.current_gait_info.time_acceleration[i]=gp.current_gait_info.actual_velocity[i]/
                        gp.current_gait_info.actual_acceleration[i];
                gp.current_gait_info.time_deceleration[i]=gp.current_gait_info.actual_velocity[i]/
                        gp.current_gait_info.actual_deceleration[i];
                gp.current_gait_info.time_constant[i]=
                        (absDeltaPos[i]-gp.current_gait_info.time_no_constant_speed[i])/
                        gp.current_gait_info.actual_velocity[i];

            }
            // eliminate nan
            for(int i=0;i<3;i++)
            {
                if(absDeltaPos[i]<0.001)
                {
                    gp.current_gait_info.time_acceleration[i]=0;
                    gp.current_gait_info.time_constant[i]=0;
                    gp.current_gait_info.time_deceleration[i]=0;
                }
            }

            // calculate steps 1000 cycles per second
            gp.current_gait_info.cycles_acceleration[i]=round(1000*gp.current_gait_info.time_acceleration[i]);
            gp.current_gait_info.cycles_deceleration[i]=round(1000*gp.current_gait_info.time_deceleration[i]);
            gp.current_gait_info.cycles_constant[i]=round(1000*gp.current_gait_info.time_constant[i]);

            gp.current_gait_info.cycles_total[i]=
                    gp.current_gait_info.cycles_acceleration[i]+
                    gp.current_gait_info.cycles_deceleration[i]+
                    gp.current_gait_info.cycles_constant[i];

            // store max steps
            if(gp.current_gait_info.cycles_total[i]>gp.current_gait_info.total_steps)
            {
                gp.current_gait_info.total_steps=gp.current_gait_info.cycles_total[i];
            }

            gp.total_steps=gp.current_gait_info.total_steps;

        }

    }
    //    if(gp.currentGaitInfo.gait==GAIT_H2S)
    //    {
    //        rt_printf("#############################################################################\n");
    //        rt_printf("Information about GAIT_H2S at leg %d:\n",gp.ID);
    //        rt_printf("Total Steps: %d\n",gp.currentGaitInfo.totalSteps);
    //        rt_printf("Delta Pos: %f %f %f\n"
    //                  ,gp.currentGaitInfo.deltaPos[0]
    //                ,gp.currentGaitInfo.deltaPos[1]
    //                ,gp.currentGaitInfo.deltaPos[2]);
    //        rt_printf("Tacc:%f %f %f Tcon:%f %f %f Tdec:%f %f %f\n"
    //                  ,gp.currentGaitInfo.Tacc[0]
    //                ,gp.currentGaitInfo.Tacc[1]
    //                ,gp.currentGaitInfo.Tacc[2]
    //                ,gp.currentGaitInfo.Tcon[0]
    //                ,gp.currentGaitInfo.Tcon[1]
    //                ,gp.currentGaitInfo.Tcon[2]
    //                ,gp.currentGaitInfo.Tdec[0]
    //                ,gp.currentGaitInfo.Tdec[1]
    //                ,gp.currentGaitInfo.Tdec[2]);
    //        rt_printf("Acc:%f %f %f Vel:%f %f %f Dec:%f %f %f\n"
    //                  ,gp.currentGaitInfo.actualAcc[0]
    //                ,gp.currentGaitInfo.actualAcc[1]
    //                ,gp.currentGaitInfo.actualAcc[2]
    //                ,gp.currentGaitInfo.actualVel[0]
    //                ,gp.currentGaitInfo.actualVel[1]
    //                ,gp.currentGaitInfo.actualVel[2]
    //                ,gp.currentGaitInfo.actualDec[0]
    //                ,gp.currentGaitInfo.actualDec[1]
    //                ,gp.currentGaitInfo.actualDec[2]);
    //        rt_printf("#############################################################################");



    //    }

}

// for 6d motion
void force_gait::GaitRobot::init_gait(GaitPart<6> &gp, GaitInformation<6> nextGaitInfo, long long time)
{
    //TBD
    switch(nextGaitInfo.gait)
    {
    case GAIT_MAJOR_FORWARD:// along z axis
    {
        gp.current_gait_info=nextGaitInfo;

        gp.set_original_position(this->bodyPeLast);
        rt_printf("PeCurrent: %f %f %f \n",this->bodyPeCurrent[0],this->bodyPeCurrent[1],this->bodyPeCurrent[2]);
        //        gp.SetOriginPos(this->bodyEpLast);
        rt_printf(" body original pos:\n %f %f %f\n%f %f %f\n",
                  gp.original_position[0],
                gp.original_position[1],
                gp.original_position[2],
                gp.original_position[3],
                gp.original_position[4],
                gp.original_position[5]);
        gp.set_destiny_position(gp.current_gait_info.delta_position);
        rt_printf(" body destiny pos:\n %f %f %f\n%f %f %f",
                  gp.destiny_position[0],
                gp.destiny_position[1],
                gp.destiny_position[2],
                gp.destiny_position[3],
                gp.destiny_position[4],
                gp.destiny_position[5]);

        // 0.3 0.4 0.3 normal
        // 0.1 0.2 0.1 high load
        gp.current_gait_info.max_acceleration=0.1;
        gp.current_gait_info.max_velocity=0.1;
        gp.current_gait_info.max_deceleration=0.1;
        gp.set_start_time(time);
        gp.is_need_switching=false;
        break;
    }
    default:
        break;
    }

    // Smooth the line based on current gait info
    double absDeltaPos[6];
    double maxDeltaPos=0;
    double maxAngular=0;
    for(int i=0;i<3;i++)
    {
        // abs delta and find max delta to caclulate ratio
        absDeltaPos[i]=abs(gp.current_gait_info.delta_position[i]);
        if(absDeltaPos[i]>maxDeltaPos)
        {
            maxDeltaPos=absDeltaPos[i];
        }
    }

    if(gp.current_gait_info.gait!=GAIT_STANDSTILL)
    {
        gp.current_gait_info.total_steps=0;
    }

    if(gp.current_gait_info.gait!=GAIT_STANDSTILL)
    {
        for(int i=0;i<3;i++)
        {
            // judge direction
            if(gp.current_gait_info.delta_position[i]<0)
            {
                gp.current_gait_info.direction[i]=-1;
            }
            else
            {
                gp.current_gait_info.direction[i]=1;
            }

            gp.current_gait_info.ratio[i]=absDeltaPos[i]/maxDeltaPos;
            if(abs(maxDeltaPos)<=0.001)
            {
                gp.current_gait_info.ratio[i]=0;
            }
            // ratio could be zero, which wil affect actualAcc

            gp.current_gait_info.actual_acceleration[i]=gp.current_gait_info.max_acceleration*
                    gp.current_gait_info.ratio[i];
            gp.current_gait_info.actual_deceleration[i]=gp.current_gait_info.max_deceleration*
                    gp.current_gait_info.ratio[i];
            gp.current_gait_info.actual_velocity[i]=gp.current_gait_info.max_velocity*
                    gp.current_gait_info.ratio[i];

            //
            double TaccMAX=gp.current_gait_info.actual_velocity[i]/gp.current_gait_info.max_acceleration;
            double TdecMAX=gp.current_gait_info.actual_velocity[i]/gp.current_gait_info.max_deceleration;
            rt_printf("MAX TIME %f %f\n",TaccMAX,TdecMAX);
            gp.current_gait_info.time_no_constant_speed[i]=
                    0.5*gp.current_gait_info.actual_acceleration[i]*TaccMAX*TaccMAX+
                    0.5*gp.current_gait_info.actual_deceleration[i]*TdecMAX*TdecMAX;
            if(((double)gp.current_gait_info.direction[i]*gp.current_gait_info.delta_position[i])<
                    gp.current_gait_info.time_no_constant_speed[i])
            {
                //                rt_printf("smooth no con\n");
                gp.current_gait_info.time_acceleration[i]=
                        (sqrt(2.0*gp.current_gait_info.actual_deceleration[i]*absDeltaPos[i]))
                        /(sqrt(gp.current_gait_info.actual_acceleration[i]*gp.current_gait_info.actual_acceleration[i]+gp.current_gait_info.actual_acceleration[i]*gp.current_gait_info.actual_deceleration[i]));

                gp.current_gait_info.time_deceleration[i]=
                        (sqrt(2.0)*gp.current_gait_info.actual_acceleration[i]*sqrt(absDeltaPos[i]))
                        /(sqrt(gp.current_gait_info.actual_deceleration[i]*gp.current_gait_info.actual_acceleration[i]*(gp.current_gait_info.actual_deceleration[i]+gp.current_gait_info.actual_acceleration[i])));
                gp.current_gait_info.time_constant[i]=0;
                rt_printf("smooth no con\n %d %f \n",i,gp.current_gait_info.time_acceleration[i]);
                rt_printf("smooth no con\n %d %f \n",i,gp.current_gait_info.time_constant[i]);
                rt_printf("smooth no con\n %d %f \n",i,gp.current_gait_info.time_deceleration[i]);

            }
            else
            {
                rt_printf("smooth con\n");
                gp.current_gait_info.time_acceleration[i]=gp.current_gait_info.actual_velocity[i]/
                        gp.current_gait_info.actual_acceleration[i];
                gp.current_gait_info.time_deceleration[i]=gp.current_gait_info.actual_velocity[i]/
                        gp.current_gait_info.actual_deceleration[i];
                gp.current_gait_info.time_constant[i]=
                        (absDeltaPos[i]-gp.current_gait_info.time_no_constant_speed[i])/
                        gp.current_gait_info.actual_velocity[i];
                rt_printf("smooth con\n %d %f \n",i,gp.current_gait_info.actual_velocity[i]);
                rt_printf("smooth con\n %d %f \n",i,gp.current_gait_info.actual_acceleration[i]);
                rt_printf("smooth con\n %d %f \n",i,gp.current_gait_info.time_acceleration[i]);

            }
            if(abs(gp.current_gait_info.delta_position[i])<=0.001)// 1mm here
            {
                gp.current_gait_info.time_acceleration[i]=0;
                gp.current_gait_info.time_constant[i]=0;
                gp.current_gait_info.time_deceleration[i]=0;

            }

            // calculate steps 1000 cycles per second
            gp.current_gait_info.cycles_acceleration[i]=round(1000*gp.current_gait_info.time_acceleration[i]);
            gp.current_gait_info.cycles_deceleration[i]=round(1000*gp.current_gait_info.time_deceleration[i]);
            gp.current_gait_info.cycles_constant[i]=round(1000*gp.current_gait_info.time_constant[i]);

            gp.current_gait_info.cycles_total[i]=
                    gp.current_gait_info.cycles_acceleration[i]+
                    gp.current_gait_info.cycles_deceleration[i]+
                    gp.current_gait_info.cycles_constant[i];

            // store max steps
            if(gp.current_gait_info.cycles_total[i]>gp.current_gait_info.total_steps)
            {
                gp.current_gait_info.total_steps=gp.current_gait_info.cycles_total[i];
            }



        }
    }

    gp.total_steps=gp.current_gait_info.total_steps;
    rt_printf("\nFirst Init Body xyz time \n %d,%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n ",gp.id,
              gp.current_gait_info.time_acceleration[0],
            gp.current_gait_info.time_acceleration[1],
            gp.current_gait_info.time_acceleration[2],
            gp.current_gait_info.time_constant[0],
            gp.current_gait_info.time_constant[1],
            gp.current_gait_info.time_constant[2],
            gp.current_gait_info.time_deceleration[0],
            gp.current_gait_info.time_deceleration[1],
            gp.current_gait_info.time_deceleration[2]);
    rt_printf("\nFirst Init Body xyz time  speed distribution\n %d,%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n ",gp.id,
              gp.current_gait_info.actual_acceleration[0],
            gp.current_gait_info.actual_acceleration[1],
            gp.current_gait_info.actual_acceleration[2],
            gp.current_gait_info.actual_velocity[0],
            gp.current_gait_info.actual_velocity[1],
            gp.current_gait_info.actual_velocity[2],
            gp.current_gait_info.actual_deceleration[0],
            gp.current_gait_info.actual_deceleration[1],
            gp.current_gait_info.actual_deceleration[2]);

    rt_printf("\nFirst Init Body xyz time OTHER \n %d,%f,%f,%f\n%f,%f,%f\n%d,%d,%d\n",gp.id,
              gp.current_gait_info.time_no_constant_speed[0],
            gp.current_gait_info.time_no_constant_speed[1],
            gp.current_gait_info.time_no_constant_speed[2],
            absDeltaPos[0],
            absDeltaPos[1],
            absDeltaPos[2],
            gp.current_gait_info.cycles_deceleration[2],
            gp.current_gait_info.cycles_constant[2],
            gp.current_gait_info.cycles_acceleration[2]
            );

    // update angular information TBD this need rewrite 2015 06 29
    // currently the angular displacement is not coincide with translating.
    for(int i=3;i<6;i++)
    {
        // abs delta and find max delta to caclulate ratio
        absDeltaPos[i]=abs(gp.current_gait_info.delta_position[i]);
        if(absDeltaPos[i]>maxAngular)
        {
            maxAngular=absDeltaPos[i];
        }
    }

    //    if(gp.currentGaitInfo.gait!=GAIT_STANDSTILL)
    //    {
    //        gp.currentGaitInfo.totalSteps=0;
    //    }
    if(gp.current_gait_info.gait!=GAIT_STANDSTILL)
    {
        for(int i=3;i<6;i++)
        {
            // judge direction
            if(gp.current_gait_info.delta_position[i]<0)
            {
                gp.current_gait_info.direction[i]=-1;
            }
            else
            {
                gp.current_gait_info.direction[i]=1;
            }

            gp.current_gait_info.ratio[i]=absDeltaPos[i]/maxAngular;
            if(abs(maxAngular)<=0.001)
            {
                gp.current_gait_info.ratio[i]=0;
            }
            // ratio could be zero, which wil affect actualAcc

            gp.current_gait_info.actual_acceleration[i]=gp.current_gait_info.max_acceleration_angular*
                    gp.current_gait_info.ratio[i];
            gp.current_gait_info.actual_deceleration[i]=gp.current_gait_info.max_deceleration_angular*
                    gp.current_gait_info.ratio[i];
            gp.current_gait_info.actual_velocity[i]=gp.current_gait_info.max_velocity_angular*
                    gp.current_gait_info.ratio[i];

            //
            double TaccMAX=gp.current_gait_info.actual_velocity[i]/gp.current_gait_info.max_acceleration_angular;
            double TdecMAX=gp.current_gait_info.actual_velocity[i]/gp.current_gait_info.max_deceleration_angular;
            gp.current_gait_info.time_no_constant_speed[i]=
                    0.5*gp.current_gait_info.actual_acceleration[i]*TaccMAX*TaccMAX+
                    0.5*gp.current_gait_info.actual_deceleration[i]*TdecMAX*TdecMAX;
            if(((double)gp.current_gait_info.direction[i]*gp.current_gait_info.delta_position[i])<
                    gp.current_gait_info.time_no_constant_speed[i])
            {
                gp.current_gait_info.time_acceleration[i]=
                        (sqrt(2.0*gp.current_gait_info.actual_deceleration[i]*absDeltaPos[i]))
                        /(sqrt(gp.current_gait_info.actual_acceleration[i]*gp.current_gait_info.actual_acceleration[i]+gp.current_gait_info.actual_acceleration[i]*gp.current_gait_info.actual_deceleration[i]));

                gp.current_gait_info.time_deceleration[i]=
                        (sqrt(2.0)*gp.current_gait_info.actual_acceleration[i]*sqrt(absDeltaPos[i]))
                        /(sqrt(gp.current_gait_info.actual_deceleration[i]*gp.current_gait_info.actual_acceleration[i]*(gp.current_gait_info.actual_deceleration[i]+gp.current_gait_info.actual_acceleration[i])));
                gp.current_gait_info.time_constant[i]=0;
            }
            else
            {
                gp.current_gait_info.time_acceleration[i]=gp.current_gait_info.actual_velocity[i]/
                        gp.current_gait_info.actual_acceleration[i];
                gp.current_gait_info.time_deceleration[i]=gp.current_gait_info.actual_velocity[i]/
                        gp.current_gait_info.actual_deceleration[i];
                gp.current_gait_info.time_constant[i]=
                        (absDeltaPos[i]-gp.current_gait_info.time_no_constant_speed[i])/
                        gp.current_gait_info.actual_velocity[i];

            }
            if(abs(gp.current_gait_info.delta_position[i])<=0.001)// 1mm here
            {
                gp.current_gait_info.time_acceleration[i]=0;
                gp.current_gait_info.time_constant[i]=0;
                gp.current_gait_info.time_deceleration[i]=0;

            }

            // calculate steps 1000 cycles per second
            gp.current_gait_info.cycles_acceleration[i]=round(1000*gp.current_gait_info.time_acceleration[i]);
            gp.current_gait_info.cycles_deceleration[i]=round(1000*gp.current_gait_info.time_deceleration[i]);
            gp.current_gait_info.cycles_constant[i]=round(1000*gp.current_gait_info.time_constant[i]);

            gp.current_gait_info.cycles_total[i]=
                    gp.current_gait_info.cycles_acceleration[i]+
                    gp.current_gait_info.cycles_deceleration[i]+
                    gp.current_gait_info.cycles_constant[i];

            // store max steps
            if(gp.current_gait_info.cycles_total[i]>gp.current_gait_info.total_steps)
            {
                gp.current_gait_info.total_steps=gp.current_gait_info.cycles_total[i];
            }



        }


    }
    gp.total_steps=gp.current_gait_info.total_steps;
    rt_printf("First Init Body angular time \n %d,%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n ",gp.id,
              gp.current_gait_info.actual_acceleration[3],
            gp.current_gait_info.actual_acceleration[4],
            gp.current_gait_info.actual_acceleration[5],
            gp.current_gait_info.actual_deceleration[3],
            gp.current_gait_info.actual_deceleration[4],
            gp.current_gait_info.actual_deceleration[5],
            gp.current_gait_info.actual_velocity[3],
            gp.current_gait_info.actual_velocity[4],
            gp.current_gait_info.actual_velocity[5]);

}


void force_gait::GaitRobot::post_process(Aris::RT_CONTROL::CMachineData &data)
{
    this->model_data_to_motor_data(this->command_model_data_mapped,
                                   this->command_motor_data_mapped);
    for(int i=0;i<MOTORS_NUM;i++)
    {
        if(data.isMotorHomed[i])
        {
            data.commandData[i]=this->command_motor_data_mapped[MapPhyToAbs[i]];
        }
        else
        {
            //should not be changed
            // this only happened in HM1D,HM2D,H2S1,H2S2
            // any state change to these state should SetStandStillData
            data.commandData[i].Position=this->standstill_machine_data.commandData[i].Position;
        }
    }

}

void force_gait::GaitRobot::safety(Aris::RT_CONTROL::CMachineData &data)
{
    //    return ;
    bool IsPrint=false;
    int MaxSpeed=65536*5;
    //for safe
    for(int i=0;i<18;i++)
    {
        if(data.isMotorHomed[i])
        {
            if(data.commandData[i].Position-data.feedbackData[i].Position>MaxSpeed)
            {
                rt_printf("Not safe motor ID:%d cmd:%d fdk:%d\n",i,data.commandData[i].Position,
                          data.feedbackData[i].Position);

                this->set_standstill_data(data);
                this->next_motion=ERS_RNST;
                IsPrint=true;

            }
            else if(data.commandData[i].Position-data.feedbackData[i].Position<-MaxSpeed)
            {
                rt_printf("Not safe motor ID:%d cmd:%d fdk:%d\n",i,data.commandData[i].Position,
                          data.feedbackData[i].Position);
                this->set_standstill_data(data);
                this->next_motion=ERS_RNST;
                IsPrint=true;
            }


        }


    }

    if(IsPrint)
    {
        for(int i=0;i<6;i++)
        {
            rt_printf("Leg ID: %d tp: %f %f %f fp: %f %f %f\n",i
                      ,this->gait_part_leg[i].next_target_position[0]
                    ,this->gait_part_leg[i].next_target_position[1]
                    ,this->gait_part_leg[i].next_target_position[2]
                    ,this->leg_status[i].foot_position_ref_coord[0]
                    ,this->leg_status[i].foot_position_ref_coord[1]
                    ,this->leg_status[i].foot_position_ref_coord[2]);
        }
        rt_printf("Prismatic pos:%f %f %f\n",
                  this->leg_status[0].prismatic_position[0]
                ,this->leg_status[0].prismatic_position[1]
                ,this->leg_status[0].prismatic_position[2]);
        rt_printf("Prismatic pos:%f %f %f\n",
                  this->leg_status[2].prismatic_position[0]
                ,this->leg_status[2].prismatic_position[1]
                ,this->leg_status[2].prismatic_position[2]);

        rt_printf("feedback pos\n");
        for(int i=0;i<18;i++)
        {
            //            rt_printf("%d \t",data.feedbackData[MapAbsToPhy[i]].Position);
            rt_printf("%d \t",data.feedbackData[i].Position);
        }
        rt_printf("\n");

        rt_printf("feedbackModelDataMappedFiltered \n");
        for(int i=0;i<18;i++)
        {
            rt_printf("%f \t",this->feedback_model_data_mapped_filtered[i].position);
        }
        rt_printf("\n");

        rt_printf("feedbackModelDataMapped \n");
        for(int i=0;i<18;i++)
        {
            rt_printf("%f \t",this->feedback_model_data_mapped[i].position);
        }
        rt_printf("\n");

        rt_printf("feedbackMotorDataMapped \n");
        for(int i=0;i<18;i++)
        {
            rt_printf("%d \t",this->feedback_motor_data_mapped[i].Position);
        }
        rt_printf("\n");


    }



}

void force_gait::GaitRobot::set_standstill_data(Aris::RT_CONTROL::CMachineData &data)
{
    this->standstill_machine_data=data;
    for(int i=0;i<MOTORS_NUM;i++)
    {
        this->standstill_machine_data.commandData[i].Position=data.feedbackData[i].Position;
        this->standstill_machine_data.commandData[i].Velocity=data.feedbackData[i].Velocity;
        this->standstill_machine_data.commandData[i].Torque=data.feedbackData[i].Torque;
    }

}

ERofoGaitSTA force_gait::GaitRobot::robot_motion_maker(ERofoGait cmd)
{
    ERofoGaitSTA ret=ERS_NULL;
    robotStateMachine.SetOrder(cmd);
    ret=this->robotStateMachine.GetNextState();
    return ret;
}

void force_gait::GaitRobot::debug_print_ee_actual_target(long long *time)
{
    if(time!=NULL)
        rt_printf("##### DEBUG INFO BEGIN ##### time: %lld\n",*time);
    else
        rt_printf("##### DEBUG INFO BEGIN #####\n");
    rt_printf("actual body position:\t %f %f %f %f %f %f \n"
              ,BodyPeG[0]
            ,BodyPeG[1]
            ,BodyPeG[2]
            ,BodyPeG[3]
            ,BodyPeG[4]
            ,BodyPeG[5]);
    rt_printf("destin body position:\t %f %f %f %f %f %f \n"
              ,gait_part_body.destiny_position[0]
            ,gait_part_body.destiny_position[1]
            ,gait_part_body.destiny_position[2]
            ,gait_part_body.destiny_position[3]
            ,gait_part_body.destiny_position[4]
            ,gait_part_body.destiny_position[5]);
    rt_printf("target body position:\t %f %f %f %f %f %f \n"
              ,gait_part_body.next_target_position[0]
            ,gait_part_body.next_target_position[1]
            ,gait_part_body.next_target_position[2]
            ,gait_part_body.next_target_position[3]
            ,gait_part_body.next_target_position[4]
            ,gait_part_body.next_target_position[5]);


    char LegName[6][3]={"LF","LM","LB","RF","RM","RB"};
    for(int i=0;i<6;i++)
    {
        rt_printf("%s:\n a: %f %f %f\n d: %f %f %f\n t: %f %f %f\n"
                  ,LegName[i]
                  ,leg_status[i].foot_postion_abs_coord[0]
                ,leg_status[i].foot_postion_abs_coord[1]
                ,leg_status[i].foot_postion_abs_coord[2]
                ,gait_part_leg[i].destiny_position[0]
                ,gait_part_leg[i].destiny_position[1]
                ,gait_part_leg[i].destiny_position[2]
                ,gait_part_leg[i].next_target_position[0]
                ,gait_part_leg[i].next_target_position[1]
                ,gait_part_leg[i].next_target_position[2]);

    }
    rt_printf("##### DEBUG INFO END #####\n");


}

// this function should be used before FirstInitGait
void force_gait::GaitRobot::debug_print_gait_part_state(long long *time=NULL)
{

//    char LEG_NAME[6][3]={"LF","LM","LB","RF","RM","RB"};
//    char GAIT_NAME[19][80]={
//        "GAIT_NONE",
//        "GAIT_POWEROFF",
//        "GAIT_STOP",
//        "GAIT_ENABLE",
//        "GAIT_HOME",
//        "GAIT_H2S",
//        "GAIT_STANDSTILL",
//        "GAIT_MAJOR_UP",
//        "GAIT_MINOR_UP",
//        "GAIT_MAJOR_FORWARD",
//        "GAIT_MINOR_FORWARD",
//        "GAIT_MAJOR_BACKWARD",
//        "GAIT_MINOR_BACKWARD",
//        "GAIT_MAJOR_DOWN",
//        "GAIT_MINOR_DOWN",
//        "GAIT_MINOR_UP_BACKWARD",
//        "GAIT_STRAIGHT",
//        "GAIT_LOCAL_CYCLE_FOREWARD",
//        "GAIT_LOCAL_CYCLE_BACKWARD"
//    };

    int if_has_current_step_1=0;
    for(int i=0;i<6;i++)
    {
        if(gait_part_leg[i].current_step==1)
            if_has_current_step_1++;
    }
    if(if_has_current_step_1==0)
        return;

    if(time!=NULL)
        rt_printf("##### DEBUG INFO BEGIN ##### time: %lld\n",*time);
    else
        rt_printf("##### DEBUG INFO BEGIN #####\n");

    for(int i=0;i<6;i++)
    {
        // currentStep==0 is in the last step of the last gait
        if(gait_part_leg[i].current_step==1)
        {
            rt_printf("%s first step in gait %s:\n"
                      ,LEG_NAME[gait_part_leg[i].id]
                    ,GAIT_NAME[gait_part_leg[i].current_gait_info.gait]);
            rt_printf("   actual: %f %f %f\n  destiny: %f %f %f\n original: %f %f %f\n   target: %f %f %f\n"
                      ,leg_status[i].foot_postion_abs_coord[0]
                    ,leg_status[i].foot_postion_abs_coord[1]
                    ,leg_status[i].foot_postion_abs_coord[2]
                    ,gait_part_leg[i].destiny_position[0]
                    ,gait_part_leg[i].destiny_position[1]
                    ,gait_part_leg[i].destiny_position[2]
                    ,gait_part_leg[i].original_position[0]
                    ,gait_part_leg[i].original_position[1]
                    ,gait_part_leg[i].original_position[2]
                    ,gait_part_leg[i].next_target_position[0]
                    ,gait_part_leg[i].next_target_position[1]
                    ,gait_part_leg[i].next_target_position[2]);

        }
    }
    rt_printf("##### DEBUG INFO END #####\n");
}
