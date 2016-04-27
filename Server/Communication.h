#ifndef COMMUNICATION_H
#define COMMUNICATION_H

//#define EROFOMACMD_OFFSET 0
//enum ERofoMaCMD
//{


//};

//#define EROFOMASTA_OFFSET 0

//enum ERofoMaSTA
//{

//};

// The state machine for the whole robot is defined here
// th state machine for each leg is defined in Gait.h

#define EROFOGAIT_OFFSET 4000
#define ERG_NUM 16



enum ERofoGait
{
    ERG_NULL=EROFOGAIT_OFFSET,
    ERG_POWEROFF,
    ERG_STOP,
    ERG_ENABLE,

    ERG_RUNNING,

    ERG_HOME1,
    ERG_HOME2,
    ERG_H2ST1,
    ERG_H2ST2,

    ERG_RNST,//Directly switched to running without homing, not sure if it is possible
             // rnst = running and standstill

    ERG_FORWARD,
    ERG_BACKWARD,

    ERG_LEFT,
    ERG_RIGHT,

    ERG_EXPRI1,
    ERG_EXPRI2,

};


extern char ERG_NAMES[ERG_NUM][6];


#define EROFOGAITSTA_OFFSET 5000
#define ERGS_NUM 21
enum ERofoGaitSTA
{
    ERS_NULL=EROFOGAITSTA_OFFSET,

    ERS_POWEROFF,
    ERS_POWEROFFED,

    ERS_STOP,
    ERS_STOPPED,

    ERS_ENABLE,
    ERS_ENABLED,

    ERS_RUNNING,

    ERS_HME1,// LF RM LR homing   RF LM RR maintain state
    ERS_HM1D,// LF RM LR running  RF LM RR maintain state

    ERS_HME2,// RF LM RR homing   LF RM LR maintain state
    ERS_HM2D,// RF LM RR running  LF RM LR maintain state

    ERS_H2S1,
    ERS_H2S2,

    ERS_RNST,// RUNNING and STANDSTILL,  default state of the running robot

    ERS_GFWD,
    ERS_GBWD,
    ERS_GLFT,
    ERS_GRGT,

    ERS_GEX1,
    ERS_GEX2,

};

extern char ERGS_NAMES[ERGS_NUM][6];


#define ESC_OFFSET 500
#define ESC_NUM 4

enum ESysCmd
{
    ESC_NULL=ESC_OFFSET,
    ESC_STLG,
    ESC_PSLG,
    ESC_SPLG,
};



#define EMT_OFFSET 400
#define EMT_NUM 6

enum ERofoMsgID
{
    EMT_GCMD=EMT_OFFSET,// gait cmd
    EMT_GSTA,// gait sta
    EMT_DATA,// get data CMachineData
    EMT_SYSC,// system command, such as start log, stop log
    EMT_IMU_DATA,
    EMT_CU_CMD,// custom cmd
};

extern char EMT_NAMES[EMT_NUM][6];

#endif // COMMUNICATION_H


























































