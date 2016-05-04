#ifndef GAIT_H
#define GAIT_H

//#include "Hexapod_Robot.h"

#include "Robot_Type_I.h"
//#include "HexapodIV.h"
#include "Robot_Base.h"

#include <aris.h>
//#include "Aris_Control.h"
//#include "Aris_ControlData.h"
//#include "Aris_IMU.h"
#include "Filter.h"
#include <cmath>
#include "rtdk.h"
#include "Communication.h"
#include <cstring>
#include <iostream>
#include "StaMach.h"
#include "Line.h"
#include "Compatible.h"
#include "log.h"

/****************************************************************************************/
/****************************************************************************************/
//#define IMU   /********** I M U ***********/
/****************************************************************************************/
/****************************************************************************************/
extern char LEG_NAME[][3];
extern char GAIT_NAME[][80];

extern peripherals::Log robot_log;
extern LogData temperary_log_data;

using namespace Aris::RT_CONTROL;
using namespace aris::control;

#define CYCLE_INIT_TIME 1000

/*

 p1 p2 p3
  0  1  2 LF
  3  4  5 LM
  6  7  8 LR
  9 10 11 RF
 12 13 14 RM
 15 16 17 RR

*/

// now gait will also take care some low level work

const int MapAbsToPhy[18]=
{
    10,	11,	9,   //LF 1 1  0  1  2
    12,	14,	13,  //LM 2 2  3  4  5
    17,	15,	16,  //LR 3 1  6  7  8
    6,	8,	7,   //RF 4 2  9  10 11
    3,	5,	4,   //RM 5 1  12 13 14
    0,	2,	1    //RR 6 2  15 16 17
};
const int MapPhyToAbs[18]=
{
    15,	17,	16,
    12,	14,	13,
    9,	11,	10,
    2,	0,	1,
    3,	5,	4,
    7,	8,	6
};
////For Hexapod II
//const int MapAbsToPhy[18]=
//{
//        1,	0,	2,
//        4,	3,	5,
//        7,	6,	8,
//        16,	15,	17,
//        13,	12,	14,
//        10,	9,	11
//};
//const int MapPhyToAbs[18]=
//{
//    1,	0,	2,
//    4,	3,	5,
//    7,	6,	8,
//    16,	15,	17,
//    13,	12,	14,
//    10,	9,	11
//};


namespace force_gait
{

//enumerations first

//easy way in array
enum ELEGID
{
    LF=0,
    LM,
    LR,
    RF,
    RM,
    RR,
};

// first level of motion abandoned now
enum EMOTION
{
    MO_NONE, //0
    MO_H2S, //1
    MO_STANDSTILL, //2
    MO_FORWARD,// 3
    MO_STOP,//4
    //corresponding to EOliveCmd
    MO_CA_S,//5
    MO_CA_X,//6
    MO_CA_Y,//7
    MO_CA_Z,//8
    MO_CA_BS,//9
    MO_CA_BX,
    MO_CA_BY,
    MO_CA_BZ,
    MO_CA_BRX,
    MO_CA_BRY,
    MO_CA_BRZ
};

enum ECALIBSTAGE
{
    CAL_S,
    CAL_X,
    CAL_Y,
    CAL_Z,
    CAL_BX,
    CAL_BY,
    CAL_BZ,
    CAL_BRX,
    CAL_BRY,
    CAL_BRZ,
};

// intermediate level of motion
enum EOBSTACLEGAIT
{
    OG_INIT,
    OG_FIRSTMOTION,
    OG_BODYMOTION,
    OG_SECONDMOTION
};

// motion of each foot tip, atom of motion currenlty
// and this
enum EGAITS //gait for single endpoint
{
    GAIT_NONE,
    GAIT_POWEROFF,
    GAIT_STOP,
    GAIT_ENABLE,
    GAIT_HOME,
    GAIT_H2S,
    GAIT_STANDSTILL,
    GAIT_MAJOR_UP,
    GAIT_MINOR_UP,
    GAIT_MAJOR_FORWARD,// foreward
    GAIT_MINOR_FORWARD,// backward
    GAIT_MAJOR_BACKWARD,
    GAIT_MINOR_BACKWARD,
    GAIT_MAJOR_DOWN,
    GAIT_MINOR_DOWN,
    GAIT_MINOR_UP_BACKWARD,// foreward
    GAIT_MINOR_UP_FORWARD,// backward
    GAIT_STRAIGHT,

    // added for blind walking
    GAIT_LOCAL_CYCLE_FOREWARD,
    GAIT_LOCAL_CYCLE_BACKWARD,

    //added for reaction
    //direction determined by the force impulse direction
    // first stage
    // ellipse->major_down
    // ellipse->reflex->ellipse->major_down
    // second stage
    // ellipse->reflex->ellipse->cycle_foreward->cycle_backward
    GAIT_REFLEX,
    GAIT_ELLIPSE
};

enum ENORM
{
    L0,
    L1,
    L2,
    LINF
};

class Threshold
{
public:
    Threshold();
    ~Threshold();
    void set_threshold(double lo,double hi);
    bool threshold(double value);\
    void reset();
    bool is_on();
    double value;
private:
    double hi_thr;
    double lo_thr;
    bool is_on_;

};


class AbstractMotorData
{
public:
    AbstractMotorData& operator=(const AbstractMotorData& other);
    double position;
    double velocity;
    double acceleration;
    double force;
};


// store calculation result

class LegStatus
{
public:
    double force_jacobian_direct[3][3]; // used
    //if a leg is locked, then at it will remain standstill all the time
    double prismatic_position[3];// current pLen
    double prismatic_actuation_force[3];// current pForce
    double prismatic_dynamic_force[3];//
    double prismatic_external_force[3];//

    double foot_external_force_ref_coord[3];

    double foot_force_ref_coord[3];// L and rotation matrix
    double prismatic_velocity[3];
    double prismatic_acceleration[3];
    double foot_position_ref_coord[3];//current foottip pos in mainbody coordinates
    double foot_postion_abs_coord[3];//current foottip pos in ground coordinates
    int leg_id;
};




// describe properties in atom level of gait
template<int N=3,class T=double>
class GaitInformation
{
public:
    GaitInformation();
    ~GaitInformation();
    GaitInformation<N,T>& operator =(const GaitInformation<N,T>& other);

    template<int N2>
    GaitInformation<N,T>& operator =(const GaitInformation<N2,T>& other);


    const int length=N;
    //    const int dimension=N;
    T delta_position[N];
    T destiny_position[N];
    T velocity[N];
    T acceleration[N];
    T deceleration[N];
    int total_steps;
    bool is_absolute;
    EGAITS gait;

    // for smooth line

    // set before first init gait
    // we can set this info in initializer function
    // m/s^2
    double max_acceleration=0.1;
    double max_deceleration=0.1;
    double max_velocity=0.1;
    // 0.1 0.3 0.2 big dec will cause forward start wrong signal?
    // 0.1 0.2 0.2 still forward has wrong signal
    // it seems MAJOR_UP has huge position error Log_2015_04_19_16_18_Sun.botlog
    // Let's change the MAJOR_UP speed property

    double max_acceleration_angular=3.1415926/90.0*5;
    double max_deceleration_angular=3.1415926/90.0*5;
    double max_velocity_angular=3.1415926/90.0*5;

    int hertz=1000;


    //  calculated  after  first init gait
    // put it here because this info is also independant with
    // the global coordinates
    double actual_acceleration[N];
    double actual_deceleration[N];
    double actual_velocity[N];

    T time_no_constant_speed[N];
    double time_acceleration[N]={0,0,0};
    double time_deceleration[N];
    double time_constant[N];
    int direction[N];

    double ratio[N];


    int cycles_acceleration[N];
    int cycles_deceleration[N];
    int cycles_constant[N];

    int cycles_total[N];
    int steps_total=0;




};



template<int N,class T>
GaitInformation<N,T>::GaitInformation()
{
    memset(delta_position,0,length*sizeof(T));
    memset(velocity,0,length*sizeof(T));
    memset(acceleration,0,length*sizeof(T));
    memset(deceleration,0,length*sizeof(T));
    memset(destiny_position,0,length*sizeof(T));
    total_steps=0;
    gait=GAIT_NONE;
    is_absolute=false;
}

template<int N,class T>
GaitInformation<N,T>::~GaitInformation()
{
    //do nothing here
}


template<int N,class T>
GaitInformation<N,T> &GaitInformation<N, T>::operator =(const GaitInformation<N,T>& other)
{
    memcpy(this,&other,sizeof(*this));
    return *this;
}

// realize =
template<int N,class T>
template<int N2>
GaitInformation<N,T>& GaitInformation<N,T>::operator =(const GaitInformation<N2,T>& other)
{
    //std::cout<<"unequal"<<std::endl;
    if(N==3&&N2==6)
    {
        // last three elements of 6d is 3d transition
        for(int i=0;i<3;i++)
        {
            this->acceleration[i]=other.acceleration[i];
            this->deceleration[i]=other.acceleration[i];
            this->delta_position[i]=other.delta_position[i];
            this->destiny_position[i]=other.destiny_position[i];
            this->velocity[i]=other.velocity[i];

        }
        this->is_absolute=other.is_absolute;
        this->gait=other.gait;
        this->total_steps=other.total_steps;

    }
    else if(N==6&&N2==3)
    {
        // last three elements of 6d is 3d transition
        for(int i=0;i<3;i++)
        {
            this->acceleration[i+3]=0.0;
            this->deceleration[i+3]=0.0;
            this->delta_position[i+3]=0.0;
            this->destiny_position[i+3]=0.0;
            this->velocity[i+3]=0.0;

            this->acceleration[i]=other.acceleration[i];
            this->deceleration[i]=other.acceleration[i];
            this->delta_position[i]=other.delta_position[i];
            this->destiny_position[i]=other.destiny_position[i];
            this->velocity[i]=other.velocity[i];

        }
        this->is_absolute=other.is_absolute;
        this->gait=other.gait;
        this->total_steps=other.total_steps;

    }
    else
    {
        //if demension is not right then unchange anything
        // this shuold not happen in our program.
        return *this;
    }
}


//CGaitX could be bodyGait or legGait
template<int N=3,class T=double>
class GaitPart
{
public:
    GaitPart();
    ~GaitPart();
    const int length=N;
    //    const int dimension=N;
    // functions
    template<int Nt=3,class Tt=double>
    void set_gait(GaitInformation<Nt,Tt> immediate_gait,GaitInformation<Nt,Tt> nex_pos,GaitInformation<Nt,Tt> nex_fce);
    void get_next_target_position(double pos[N],long long int time, LegStatus legstate);
    void set_original_position(double pos[N]);
    void set_destiny_position(double pos[N]);
    void set_total_steps(int step);
    void set_start_time(long long int time);

public:// Gait info

    //pointer to calculated value in the CGaitRobot
    T original_position[N];
    T destiny_position[N];
    T next_target_position[N];

    GaitInformation<N> current_gait_info;
    GaitInformation<N> next_gait_info;
    GaitInformation<N> next_gait_info_by_position;
    GaitInformation<N> next_gait_info_by_force;


    //added for blind walking
    //    CSemiCycleExt localCycle;
    CSemiEllipseExt local_cycle;
    CSmoothParameter local_smooth_param;

    bool is_need_switching;
    long long int start_time;
    int current_step;
    int total_steps;
    //take as a reference point
    double gait_ref_position_body_coord[3];
    //*PosLimit[x,y,z][up,down]
    double position_limit[3][2];

    bool is_active=false;

    int id;

    // we need three parameters to identify the limbs here,
    // but this make things awkward, need improve this in the future
    // when the part is the body,this will be totally useless.
    int limb_id[3];// Only for leg part

    //threshold
    Threshold threshold_y_positive;
    Threshold threshold_z_positive;
    Threshold threshold_z_negative;

    //limit
    //experiment shows 600 is too small
    // this is strange, because former experiment this threshold is ok
    // but to solve this problem, we use 800.0 instead
    double limit_y_positive_hi=1300.0;
    double limit_y_positive_lo=100.0;

    //here we meet the same phenomenon, 200.0 isss too small now
    // this is about friction force, now we use 300.0 instead
    double limit_z_positive_hi=300.0;
    double limit_z_positive_lo=100.0;

    double limit_z_negative_hi=-100.0;
    double limit_z_negative_lo=-300.0;

    //****static gait reference library******//
    GaitInformation<> gl_H2S;

    GaitInformation<> gl_MAJOR_UP;
    GaitInformation<> gl_MAJOR_BACKWARD;
    GaitInformation<> gl_MAJOR_DOWN;
    GaitInformation<> gl_MAJOR_FORWARD;

    GaitInformation<> gl_MINOR_UP;
    GaitInformation<> gl_MINOR_BACKWARD;
    GaitInformation<> gl_MINOR_DOWN;
    GaitInformation<> gl_MINOR_FORWARD;

    GaitInformation<> gl_MINOR_UP_BACKWARD;
    GaitInformation<> gl_MINOR_UP_FORWARD;
    GaitInformation<> gl_STANDSTILL;

    GaitInformation<> gl_LOCAL_CYCLE_FOREWARD;
    GaitInformation<> gl_LOCAL_CYCLE_BACKWARD;




    //for body motion
    GaitInformation<6> gb_STANDSTILL;
    GaitInformation<6> gb_MAJOR_UP;
    GaitInformation<6> gb_MAJOR_DOWN;
    GaitInformation<6> gb_MAJOR_FORWARD;
    GaitInformation<6> gb_MAJOR_BACKWARD;

    //for calibration




};

template<int N,class T>
GaitPart<N,T>::GaitPart()
{
    id=-1;
    is_need_switching=false;
    current_gait_info.gait=GAIT_NONE;
    next_gait_info.gait=GAIT_STANDSTILL;
    next_gait_info_by_force.gait=GAIT_STANDSTILL;
    next_gait_info_by_position.gait=GAIT_STANDSTILL;
    start_time=0;
    //Set some array and pointers
    for(int i=0;i<N;i++)
    {
        destiny_position[i]=0;
        original_position[i]=0;
        next_target_position[i]=0;
    }
    //Set gait library
    gl_STANDSTILL.gait=GAIT_STANDSTILL;
    gl_STANDSTILL.delta_position[0]=0;
    gl_STANDSTILL.delta_position[1]=0;
    gl_STANDSTILL.delta_position[2]=0;
    gl_STANDSTILL.total_steps=1000000000;

    gl_H2S.gait=GAIT_H2S;
    gl_H2S.delta_position[0]=0;
    gl_H2S.delta_position[1]=-0.30;
    gl_H2S.delta_position[2]=0;
    gl_H2S.total_steps=30000;
    for(int i=0;i<3;i++)
    {
        gl_H2S.velocity[i]=gl_H2S.delta_position[i]/((double)gl_H2S.total_steps);
    }


    for(int i=0;i<3;i++)
    {
        gl_H2S.velocity[i]=gl_H2S.delta_position[i]/((double)gl_H2S.total_steps);
    }
    gl_MAJOR_UP.gait=GAIT_MAJOR_UP;
    gl_MAJOR_UP.delta_position[0]=0;
    gl_MAJOR_UP.delta_position[1]=0.05;
    gl_MAJOR_UP.delta_position[2]=0;
    gl_MAJOR_UP.total_steps=4000;
    for(int i=0;i<3;i++)
    {
        gl_MAJOR_UP.velocity[i]=gl_MAJOR_UP.delta_position[i]/((double)gl_MAJOR_UP.total_steps);
    }

    gl_MINOR_UP.gait=GAIT_MINOR_UP;
    gl_MINOR_UP.delta_position[0]=0;
    gl_MINOR_UP.delta_position[1]=0.05;
    gl_MINOR_UP.delta_position[2]=0;
    gl_MINOR_UP.total_steps=2000;
    for(int i=0;i<3;i++)
    {
        gl_MINOR_UP.velocity[i]=gl_MINOR_UP.delta_position[i]/((double)gl_MINOR_UP.total_steps);
    }

    gl_MAJOR_DOWN.gait=GAIT_MAJOR_DOWN;
    gl_MAJOR_DOWN.delta_position[0]=0;
    gl_MAJOR_DOWN.delta_position[1]=-0.05;
    gl_MAJOR_DOWN.delta_position[2]=0;
    gl_MAJOR_DOWN.total_steps=4000;
    for(int i=0;i<3;i++)
    {
        gl_MAJOR_DOWN.velocity[i]=gl_MAJOR_DOWN.delta_position[i]/((double)gl_MAJOR_DOWN.total_steps);
    }

    gl_MAJOR_FORWARD.gait=GAIT_MAJOR_FORWARD;
    gl_MAJOR_FORWARD.delta_position[0]=0;
    gl_MAJOR_FORWARD.delta_position[1]=0;
    gl_MAJOR_FORWARD.delta_position[2]=-0.20;
    gl_MAJOR_FORWARD.total_steps=8000;
    for(int i=0;i<3;i++)
    {
        gl_MAJOR_FORWARD.velocity[i]=gl_MAJOR_FORWARD.delta_position[i]/((double)gl_MAJOR_FORWARD.total_steps);
    }

    gl_MAJOR_BACKWARD.gait=GAIT_MAJOR_BACKWARD;
    gl_MAJOR_BACKWARD.delta_position[0]=0;
    gl_MAJOR_BACKWARD.delta_position[1]=0;
    gl_MAJOR_BACKWARD.delta_position[2]=0.20;
    gl_MAJOR_BACKWARD.total_steps=8000;
    for(int i=0;i<3;i++)
    {
        gl_MAJOR_BACKWARD.velocity[i]=gl_MAJOR_BACKWARD.delta_position[i]/((double)gl_MAJOR_BACKWARD.total_steps);
    }


    gl_MINOR_UP_BACKWARD.gait=GAIT_MINOR_UP_BACKWARD;
    gl_MINOR_UP_BACKWARD.delta_position[0]=0;
    gl_MINOR_UP_BACKWARD.delta_position[1]=0.05;//y
    gl_MINOR_UP_BACKWARD.delta_position[2]=0.03;//z
    gl_MINOR_UP_BACKWARD.total_steps=2000;
    for(int i=0;i<3;i++)
    {
        gl_MINOR_UP_BACKWARD.velocity[i]=gl_MINOR_UP_BACKWARD.delta_position[i]/((double)gl_MINOR_UP_BACKWARD.total_steps);
    }

    gl_MINOR_UP_FORWARD.gait=GAIT_MINOR_UP_FORWARD;
    gl_MINOR_UP_FORWARD.delta_position[0]=0;
    gl_MINOR_UP_FORWARD.delta_position[1]=0.05;//y
    gl_MINOR_UP_FORWARD.delta_position[2]=-0.03;//z
    gl_MINOR_UP_FORWARD.total_steps=2000;
    for(int i=0;i<3;i++)
    {
        gl_MINOR_UP_FORWARD.velocity[i]=gl_MINOR_UP_FORWARD.delta_position[i]/((double)gl_MINOR_UP_FORWARD.total_steps);
    }


    //added for blind walking

    gl_LOCAL_CYCLE_FOREWARD.gait=GAIT_LOCAL_CYCLE_FOREWARD;
    gl_LOCAL_CYCLE_FOREWARD.total_steps=3000;

    gl_LOCAL_CYCLE_BACKWARD.gait=GAIT_LOCAL_CYCLE_BACKWARD;
    gl_LOCAL_CYCLE_BACKWARD.total_steps=3000;


    gb_MAJOR_UP.gait=GAIT_MAJOR_UP;
    this->gb_MAJOR_UP.delta_position[0]=0;
    this->gb_MAJOR_UP.delta_position[1]=0.10;
    this->gb_MAJOR_UP.delta_position[2]=0;
    this->gb_MAJOR_UP.total_steps=4000;
    for(int i=0;i<3;i++)
    {
        gb_MAJOR_UP.velocity[i]=gb_MAJOR_UP.delta_position[i]/((double)gb_MAJOR_UP.total_steps);
    }
    gb_MAJOR_DOWN=gl_MAJOR_DOWN;
    gb_MAJOR_FORWARD=gl_MAJOR_FORWARD;
    gb_MAJOR_UP=gl_MAJOR_UP;
    gb_STANDSTILL=gl_STANDSTILL;





}

template<int N,class T>
GaitPart<N,T>::~GaitPart()
{


}

template<int N,class T>
template<int Nt,class Tt>
void GaitPart<N,T>::set_gait(GaitInformation<Nt,Tt> immediate_gait,GaitInformation<Nt,Tt> nex_pos,GaitInformation<Nt,Tt> nex_fce)
{
    if(N!=Nt)
    {
        rt_printf("Error in SetGait");
        return 0;
    }
    else
    {
        this->current_gait_info=immediate_gait;
        this->next_gait_info_by_position=nex_pos;
        this->next_gait_info_by_force=nex_fce;

    }

}

template<int N,class T>
void GaitPart<N,T>::set_original_position(double pos[N])
{
    for(int i=0;i<N;i++)
    {
        this->original_position[i]=pos[i];
    }

}

// 2015-06-23
// The current RunGait is the calculate based on the cooridnate frame G
// which will cause a sudden change if the robot fall and the gyroscope generates new body position
// The sudden change of the position is a dangerous situation for the present gait planning.
// First thing is describe the prolem carefully.
template<int N,class T>
void GaitPart<N,T>::set_destiny_position(double pos[])
{
    for(int i=0;i<N;i++)
    {
        this->destiny_position[i]=this->original_position[i]+pos[i];
    }

}

template<int N,class T>
void GaitPart<N,T>::set_total_steps(int step)
{
    this->total_steps=step;
}


template<int N,class T>
void GaitPart<N,T>::set_start_time(long long int time)
{
    this->start_time=time;
}

template<int N,class T>
void GaitPart<N,T>::get_next_target_position(double pos[N],long long int time, LegStatus legstate)
{
    this->current_step=time-this->start_time;
    if(N==3)
    {
        if(this->current_gait_info.gait==GAIT_STANDSTILL)
        {
            for(int i=0;i<N;i++)
            {
                pos[i]=this->original_position[i];
            }

        }
        else if(this->current_gait_info.gait==GAIT_LOCAL_CYCLE_FOREWARD)
        {
            double theta;

            this->local_smooth_param.MapTimeToParam(
                        ((double)this->current_step)/((double)this->total_steps),theta);
            this->local_cycle.SetCurrentParam(theta);
            this->local_cycle.GetCurvePoint(
                        this->gl_LOCAL_CYCLE_FOREWARD.delta_position[2],
                    this->gl_LOCAL_CYCLE_FOREWARD.delta_position[1]);
            pos[0]=this->original_position[0];
            pos[1]=this->original_position[1]+this->gl_LOCAL_CYCLE_FOREWARD.delta_position[1];
            pos[2]=this->original_position[2]+this->gl_LOCAL_CYCLE_FOREWARD.delta_position[2];
            if(current_step<10||(current_step<1000&&current_step%20==0&&(this->id==0||this->id==3)))
            {
                rt_printf("%d first cycle forward delta %f %f %f\n%f %f %f\n",
                          current_step,
                          gl_LOCAL_CYCLE_FOREWARD.delta_position[0],
                        gl_LOCAL_CYCLE_FOREWARD.delta_position[1],
                        gl_LOCAL_CYCLE_FOREWARD.delta_position[2],
                        pos[0],pos[1],pos[2]);
            }

        }
        else if(this->current_gait_info.gait==GAIT_LOCAL_CYCLE_BACKWARD)
        {
            double theta;

            this->local_smooth_param.MapTimeToParam(
                        (double)this->current_step/(double)this->total_steps,theta);
            this->local_cycle.SetCurrentParam(theta);
            this->local_cycle.GetCurvePoint(
                        this->gl_LOCAL_CYCLE_BACKWARD.delta_position[2],
                    this->gl_LOCAL_CYCLE_BACKWARD.delta_position[1]);
            pos[0]=this->original_position[0];
            pos[1]=this->original_position[1]+this->gl_LOCAL_CYCLE_BACKWARD.delta_position[1];
            pos[2]=this->original_position[2]+this->gl_LOCAL_CYCLE_BACKWARD.delta_position[2];

        }
        else
        {
            for(int i=0;i<N;i++)
            {
                //Add line smoother.......................................TBD

                //                pos[i]=this->originEndPos[i]+
                //                        (this->destEndPos[i]-this->originEndPos[i])/
                //                        ((double)this->totalSteps)*((double)this->currentStep);
                // here <= may cause problems
                // when Tacc may equals -nan
                if(this->current_step < this->current_gait_info.cycles_acceleration[i])
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*
                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            (this->current_step/1000.0)*
                            (this->current_step/1000.0);
                }
                else if(this->current_step < (this->current_gait_info.cycles_acceleration[i]+
                                              this->current_gait_info.cycles_constant[i]))
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*
                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]+

                            this->current_gait_info.direction[i]*
                            (this->current_step-this->current_gait_info.cycles_acceleration[i])/1000.0*
                            this->current_gait_info.actual_velocity[i];

                }
                else if(this->current_step < this->current_gait_info.cycles_total[i])
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*

                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]+

                            this->current_gait_info.direction[i]*
                            this->current_gait_info.time_constant[i]*
                            this->current_gait_info.actual_velocity[i]+

                            this->current_gait_info.direction[i]*
                            (this->current_step/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            this->current_gait_info.actual_velocity[i]-

                            (double)this->current_gait_info.direction[i]*
                            0.5*this->current_gait_info.actual_deceleration[i]*
                            (this->current_step/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            (this->current_step/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i]);



                }
                else
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*

                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]+

                            (double)this->current_gait_info.direction[i]*
                            this->current_gait_info.time_constant[i]*
                            this->current_gait_info.actual_velocity[i]+

                            (double)this->current_gait_info.direction[i]*
                            (this->current_gait_info.cycles_total[i]/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            this->current_gait_info.actual_velocity[i]-

                            (double)this->current_gait_info.direction[i]*
                            0.5*this->current_gait_info.actual_deceleration[i]*
                            (this->current_gait_info.cycles_total[i]/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            (this->current_gait_info.cycles_total[i]/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i]);

                }



            }
        }
        //transitions
        if(this->current_step==this->total_steps)
        {
            switch(this->current_gait_info.gait)
            {
            case GAIT_STRAIGHT:
                rt_printf("GAIT_STRAIGHT pos end\n");
                this->next_gait_info=this->gl_STANDSTILL;
                this->next_gait_info_by_force=this->gl_STANDSTILL;
                this->next_gait_info_by_position=this->gl_STANDSTILL;
                this->is_need_switching=true;
                break;

            case GAIT_MAJOR_UP:
                //this->m_CurrentGaitInfo=this->gait_STANDSTILL;
                //this->SetOriginPos(this->m_DestEndPos);
                rt_printf("GAIT_MAJOR_UP pos end %s\n",LEG_NAME[id]);
                this->next_gait_info=this->gl_MAJOR_BACKWARD;
                this->next_gait_info_by_force=this->gl_MINOR_UP_FORWARD;
                this->next_gait_info_by_position=this->gl_MAJOR_DOWN;
                this->is_need_switching=true;
                break;
            case GAIT_MAJOR_FORWARD:
                rt_printf("GAIT_MAJOR_FORWARD pos end %s\n",LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_position;
                this->next_gait_info_by_force=this->gl_STANDSTILL;

                //added for blind walking after MAJOR_DOWN
                this->next_gait_info_by_force=this->gl_LOCAL_CYCLE_FOREWARD;

                this->next_gait_info_by_position=this->gl_STANDSTILL;
                this->is_need_switching=true;
                break;
            case GAIT_MAJOR_BACKWARD:
                rt_printf("GAIT_MAJOR_BACKWARD pos end %s\n",LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_position;
                this->next_gait_info_by_force=this->gl_STANDSTILL;

                //added for blind walking after MAJOR_DOWN
                //you should init gl_LOCAL_CYCLE_BACKWARD here,do it after 2016-04-30
                this->next_gait_info_by_force=this->gl_LOCAL_CYCLE_BACKWARD;

                this->next_gait_info_by_position=this->gl_STANDSTILL;
                this->is_need_switching=true;
                break;

            case GAIT_MAJOR_DOWN:
                rt_printf("GAIT_MAJOR_DOWN pos end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);
                if(this->threshold_z_negative.is_on())
                {
                    rt_printf("Not edge %f\n",this->threshold_z_negative.value);
                }
                else
                {
                    rt_printf("Edge %f\n",this->threshold_z_negative.value);
                }

                this->next_gait_info=this->next_gait_info_by_position;

                /* commented for 805 cut version
                this->nextGaitInfo=this->gl_LOCAL_CYCLE_FOREWARD;
                //added for blind walking, after CYCLE_FOREWARD
                this->nextGaitInfoForce=this->gl_LOCAL_CYCLE_BACKWARD;
                this->nextGaitInfoPos=this->gl_LOCAL_CYCLE_BACKWARD;
                */
                this->next_gait_info=this->gl_STANDSTILL;
                this->next_gait_info_by_force=this->gl_STANDSTILL;
                this->next_gait_info_by_position=this->gl_STANDSTILL;

                this->is_need_switching=true;
                break;
            case GAIT_MINOR_UP_BACKWARD:
                rt_printf("GAIT_MINOR_UP_BACKWARD pos end %s\n",LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_position;
                this->next_gait_info_by_force=this->gl_MINOR_UP_BACKWARD;
                this->next_gait_info_by_position=this->gl_MAJOR_DOWN;
                this->is_need_switching=true;
                break;
            case GAIT_MINOR_UP_FORWARD:
                rt_printf("GAIT_MINOR_UP_FORWARD pos end %s\n",LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_position;
                this->next_gait_info_by_force=this->gl_MINOR_UP_FORWARD;
                this->next_gait_info_by_position=this->gl_MAJOR_DOWN;
                this->is_need_switching=true;
                break;


            case GAIT_LOCAL_CYCLE_FOREWARD:
                rt_printf("GAIT_LOCAL_CYCLE_FOREWARD pos end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);
                this->next_gait_info=gl_LOCAL_CYCLE_BACKWARD;
                this->next_gait_info_by_force=gl_STANDSTILL;
                this->next_gait_info_by_position=gl_STANDSTILL;

                this->is_need_switching=true;
                break;
            case GAIT_LOCAL_CYCLE_BACKWARD:
                rt_printf("GAIT_LOCAL_CYCLE_BACKWARD pos end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_position;
                this->next_gait_info_by_force=gl_STANDSTILL;
                this->next_gait_info_by_position=gl_STANDSTILL;
                this->is_need_switching=true;
                break;
            case GAIT_H2S:
                rt_printf("GAIT_H2S pos end %s\n",LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_position;
                this->is_need_switching=true;
                break;
            default:
                break;

            }
        }
        // following should be commented first
        else if(this->threshold_y_positive.is_on())
        {
            switch (current_gait_info.gait) {
            case GAIT_MAJOR_DOWN:

                // too short then break
                // 0.10 may be too big
                // this value shoule cope with the brick
                // or the body can be raise too high and not level
                if((this->position_limit[1][0]-legstate.foot_position_ref_coord[1])<0.04)
                {
                    break;// not transition this time
                }

                rt_printf("GAIT_MAJOR_DOWN force end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);

                if(this->threshold_z_negative.is_on())
                {
                    rt_printf("Not edge %f\n",this->threshold_z_negative.value);
                }
                else
                {
                    rt_printf("Edge %f\n",this->threshold_z_negative.value);
                }



                /* commented for 805 version
                this->nextGaitInfo=this->gl_LOCAL_CYCLE_FOREWARD;
                this->nextGaitInfoForce=this->gl_LOCAL_CYCLE_BACKWARD;
                this->nextGaitInfoPos=this->gl_LOCAL_CYCLE_BACKWARD;
                */

                /* see you later
                // here we need to take a look at the surrondings

                static double side_forces=0.0;
                static double ratio=1.0;
                static double y_positive_hi=600;
                static int left_index=0;
                static int right_index=0;

                switch(this->id)
                {
                case 0://LF
                    left_index=1;
                    right_index=3;
                    break;
                case 1://LM
                    left_index=2;
                    right_index=0;
                    break;
                case 2://LR
                    left_index=1;
                    right_index=5;
                    break;
                case 3://RF
                    left_index=0;
                    right_index=4;
                    break;
                case 4://RM
                    left_index=3;
                    right_index=5;
                    break;
                case 5://RB
                    left_index=4;
                    right_index=2;
                    break;
                default:
                    break;
                }
                side_forces=temperary_log_data.foot_tip_extern_force[left_index][1]
                        +temperary_log_data.foot_tip_extern_force[right_index][1];
                static bool is_stop;
                is_stop=false;

                if(side_forces<y_positive_hi*2*ratio)
                if(is_stop)
                {
                    this->next_gait_info=this->gl_STANDSTILL;
                    this->next_gait_info_by_force=this->gl_STANDSTILL;
                    this->next_gait_info_by_position=this->gl_STANDSTILL;
                    this->is_need_switching=true;
                }
                */

                // this is because rm fault
                if(this->id==4)
                {
                    if((temperary_log_data.foot_tip_extern_force[3][1]+
                            temperary_log_data.foot_tip_extern_force[5][1])>1000.0)
                        break;
                    else
                    {
                        rt_printf("RM fault protection: RF %f RR %f\n"
                                  ,temperary_log_data.foot_tip_extern_force[3][1]
                                ,temperary_log_data.foot_tip_extern_force[5][1]);
                    }
                }



                this->next_gait_info=this->gl_STANDSTILL;
                this->next_gait_info_by_force=this->gl_STANDSTILL;
                this->next_gait_info_by_position=this->gl_STANDSTILL;
                this->is_need_switching=true;

                break;


            case GAIT_LOCAL_CYCLE_FOREWARD:
                if(this->current_step>CYCLE_INIT_TIME)
                {
                    rt_printf("GAIT_LOCAL_CYCLE_FOREWARD force y end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);
                    this->next_gait_info=this->next_gait_info_by_force;
                    this->next_gait_info_by_force=this->gl_STANDSTILL;
                    this->next_gait_info_by_position=this->gl_STANDSTILL;
                    this->is_need_switching=true;
                    break;
                }
                break;

            case GAIT_LOCAL_CYCLE_BACKWARD:
                if(this->current_step>(this->total_steps*0.4))
                {
                    rt_printf("GAIT_LOCAL_CYCLE_BACKWARD force end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);
                    this->next_gait_info=this->next_gait_info_by_force;
                    this->next_gait_info_by_force=this->gl_STANDSTILL;
                    this->next_gait_info_by_position=this->gl_STANDSTILL;
                    this->is_need_switching=true;
                    break;
                }
                break;

            default:
                break;
            }

        }
        else if(this->threshold_z_positive.is_on())
        {
            switch (this->current_gait_info.gait) {
            case GAIT_MAJOR_FORWARD:
                rt_printf("GAIT_MAJOR_FORWARD force end %f %s\n",this->threshold_z_positive.value,LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_force;
                this->next_gait_info_by_position=this->gl_MAJOR_FORWARD;
                this->is_need_switching=true;
                break;
            case GAIT_LOCAL_CYCLE_FOREWARD:
                if(this->current_step>CYCLE_INIT_TIME)
                {
                    rt_printf("GAIT_LOCAL_CYCLE_FOREWARD force z end %f %s\n",this->threshold_y_positive.value,LEG_NAME[id]);
                    this->next_gait_info=this->gl_LOCAL_CYCLE_BACKWARD;
                    this->next_gait_info_by_force=this->gl_STANDSTILL;
                    this->next_gait_info_by_position=this->gl_STANDSTILL;
                    this->is_need_switching=true;

                }
                break;
            default:
                break;
            }
        }
        else if(!this->threshold_z_negative.is_on())
        {
            switch (this->current_gait_info.gait)
            {
            case GAIT_MAJOR_BACKWARD:
            {
                rt_printf("GAIT_MAJOR_BACKWARD force end %f %s\n",this->threshold_z_positive.value,LEG_NAME[id]);
                this->next_gait_info=this->next_gait_info_by_force;
                this->next_gait_info_by_position=this->gl_MAJOR_BACKWARD;
                this->is_need_switching=true;
                break;

            }
            default:
                break;
            }

        }



    }


    else if(N==6) // so we don't need to smooth body here also can run
    {
        if(this->current_gait_info.gait==GAIT_STANDSTILL)
        {
            for(int i=0;i<N;i++)
            {
                pos[i]=this->original_position[i];
            }

        }
        else
        {
            for(int i=0;i<N;i++)
            {
                //Add line smoother.......................................TBD

                //                pos[i]=this->originEndPos[i]+
                //                        (this->destEndPos[i]-this->originEndPos[i])/
                //                        ((double)this->totalSteps)*((double)this->currentStep);
                // here <= may cause problems
                // when Tacc may equals -nan
                if(this->current_step < this->current_gait_info.cycles_acceleration[i])
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*
                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            (this->current_step/1000.0)*
                            (this->current_step/1000.0);
                }
                else if(this->current_step < (this->current_gait_info.cycles_acceleration[i]+
                                              this->current_gait_info.cycles_constant[i]))
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*

                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]+

                            this->current_gait_info.direction[i]*
                            (this->current_step-this->current_gait_info.cycles_acceleration[i])/1000.0*
                            this->current_gait_info.actual_velocity[i];

                }
                else if(this->current_step < this->current_gait_info.cycles_total[i])
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*
                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]+

                            this->current_gait_info.direction[i]*
                            this->current_gait_info.time_constant[i]*
                            this->current_gait_info.actual_velocity[i]+

                            this->current_gait_info.direction[i]*
                            (this->current_step/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            this->current_gait_info.actual_velocity[i]-

                            (double)this->current_gait_info.direction[i]*
                            0.5*this->current_gait_info.actual_deceleration[i]*
                            (this->current_step/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            (this->current_step/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i]);



                }
                else
                {
                    pos[i]=this->original_position[i]+
                            (double)this->current_gait_info.direction[i]*

                            0.5*
                            this->current_gait_info.actual_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]*
                            this->current_gait_info.time_acceleration[i]+

                            this->current_gait_info.direction[i]*
                            this->current_gait_info.time_constant[i]*
                            this->current_gait_info.actual_velocity[i]+

                            this->current_gait_info.direction[i]*
                            (this->current_gait_info.cycles_total[i]/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            this->current_gait_info.actual_velocity[i]-

                            (double)this->current_gait_info.direction[i]*
                            0.5*this->current_gait_info.actual_deceleration[i]*
                            (this->current_gait_info.cycles_total[i]/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i])*
                            (this->current_gait_info.cycles_total[i]/1000.0-this->current_gait_info.time_acceleration[i]-this->current_gait_info.time_constant[i]);

                }

            }

            if(this->current_step==this->total_steps)
            {
                switch(this->current_gait_info.gait)
                {
                case GAIT_MAJOR_FORWARD:
                    rt_printf("Body GAIT_MAJOR_FORWARD pos end\n");
                    this->next_gait_info=this->next_gait_info_by_position;
                    this->next_gait_info_by_force=this->gb_STANDSTILL;
                    this->next_gait_info_by_position=this->gb_STANDSTILL;
                    this->is_need_switching=true;
                    break;
                case GAIT_MAJOR_BACKWARD:
                    rt_printf("Body GAIT_MAJOR_BACKWARD pos end\n");
                    this->next_gait_info=this->next_gait_info_by_position;
                    this->next_gait_info_by_force=this->gb_STANDSTILL;
                    this->next_gait_info_by_position=this->gb_STANDSTILL;
                    this->is_need_switching=true;
                    break;

                }
            }

        }


    }

}










//CGaitRobot control the gait for whole robot body
// the robot model is embeded in this class


#define RotorInertia (831.0/1000.0/100.0/100.0)
#define RATIO  (350.0*65536.0)
//for hexapod II
//#define RATIO  (350.0*4096.0)
#define MOTORS_NUM 18
#define LEG_NUM 6




// combination of Model and Gait from
class GaitRobot
{
public:
    GaitRobot();
    ~GaitRobot();
    //functions
    void init();

    //TBD
    void run_gait_robot(Robots::RobotBase &rbt,ERofoGait cmd,Aris::RT_CONTROL::CMachineData &data,const Robots::WalkParam &param);
    //TBD
    void init_gait(GaitPart<3>& gp,GaitInformation<3> nextGaitInfo,long long int time);

    void init_gait(GaitPart<6>& gp,GaitInformation<6> nextGaitInfo,long long int time);

    void post_process(Aris::RT_CONTROL::CMachineData& data);

    void safety(Aris::RT_CONTROL::CMachineData &data);

    void set_standstill_data(Aris::RT_CONTROL::CMachineData &data);
    // preserve feedback and model calculation data
    void get_standstill_command_data(Aris::RT_CONTROL::CMachineData &data);

    ERofoGaitSTA robot_motion_maker(ERofoGait cmd);

    //    Aris::Sensor::IMU imu;
private:
    double imu_euler_angles[3];




private:
    void motor_data_to_model_data(Aris::RT_CONTROL::CMotorData* inData,
                                  Aris::RT_CONTROL::CMotorData* lastData,
                                  AbstractMotorData* outData,
                                  AbstractMotorData* outDataLast);

    void model_data_to_motor_data(AbstractMotorData* inData,
                                  Aris::RT_CONTROL::CMotorData* outData);

    void filter_model_data(AbstractMotorData* inData,
                           AbstractMotorData* outData);
    void get_transform_matrix_leg_to_body(double rotx[][3],double trans[][4],const ELEGID LEG_NAME);

    //combination of Precalculation and Calculation
    void model_evaluation(Robots::RobotTypeI& robot,aris::dynamic::FloatMarker &beginMak,Aris::RT_CONTROL::CMachineData &data);

    void get_target_pee_robot(double *Pee);


public:
    ERofoGait current_command;
    ERofoGaitSTA current_motion=ERS_NULL;
    ERofoGaitSTA next_motion=ERS_NULL;

private://transplant from Gait.h


    Aris::RT_CONTROL::CMachineData standstill_machine_data;

    EOBSTACLEGAIT obstacle_gait_status_next;
    EOBSTACLEGAIT obstacle_gait_status_current;
    int obstacle_gait_count;





    long long int start_time;
#define TM_SIZE 16
    static EMOTION transition_matrix[TM_SIZE][TM_SIZE];
    StaMach::CStaMach<ERofoGait,ERofoGaitSTA,ERG_NUM,ERGS_NUM> robotStateMachine;



    double pEE[18];
    double bEE[6];
    double pIN[18];


public:
    //Robots::ROBOT Robot;
    //Robots::RobotTypeI Robot;

    double bodyPeCurrent[6];
    double bodyPeLast[6];

    //as we calculate in M coordinate, the body should always be zero vector
    double BodyPeG[6]={0,0,0,0,0,0};

private:
    double RotL2M[6][3][3];
    double TranL2M[6][4][4];
    double FceJacDir[6][3][3];




public:
    LegStatus leg_status[6];

    LegStatus body_status;
    GaitPart<> gait_part_leg[6];
    GaitPart<6> gait_part_body;

private:

    bool is_home_set[MOTORS_NUM];

    /*Six stages of data stream*/
    Aris::RT_CONTROL::CMachineData machine_data_in;

    Aris::RT_CONTROL::CMotorData feedback_motor_data[MOTORS_NUM];
    Aris::RT_CONTROL::CMotorData feedback_motor_data_last[MOTORS_NUM];

    Aris::RT_CONTROL::CMotorData feedback_motor_data_mapped[MOTORS_NUM];
    Aris::RT_CONTROL::CMotorData feedback_motor_data_mapped_last[MOTORS_NUM];


    AbstractMotorData feedback_model_data_mapped_last[MOTORS_NUM];


    AbstractMotorData feedback_model_data_mapped[MOTORS_NUM];
    AbstractMotorData feedback_model_data_mapped_filtered[MOTORS_NUM];//set

    AbstractMotorData command_model_data_mapped[MOTORS_NUM];

    Aris::RT_CONTROL::CMotorData command_motor_data_mapped[MOTORS_NUM];

    //filters
    Aris::Filter::CFilterFIR_I position_filter[MOTORS_NUM];
    Aris::Filter::CFilterFIR_I velocity_filter[MOTORS_NUM];
    Aris::Filter::CFilterFIR_I acceleration_filter[MOTORS_NUM];
    Aris::Filter::CFilterFIR_I force_filter[MOTORS_NUM];

    Aris::Filter::CFilterFIR_I foot_external_force_filter[6][3];

    // 2016-04-19 imu output exists incontinuous point, so current filter is not usable
    //    Aris::Filter::CFilterFIR_I IMURawAngleFilter[3];
    //    Aris::Filter::CFilterFIR_I IMURawVelFilter[3];
    //    Aris::Filter::CFilterFIR_I IMURawAccFilter[3];


public://IMU

    double online_angle[3];
    double online_angleVel[3];
    double online_linearAcc[3];


public://debug tools
    void debug_print_ee_actual_target(long long int *time = NULL);
    void debug_print_gait_part_state(long long *time);

private:

    int wait_time=200;


};




}





#endif
