#ifndef COMPATIBLE_H
#define COMPATIBLE_H

#define ACTUAL_MOTOR_NUMBER 18
#define AXIS_NUMBER 18

#define FREQUENCY_MAIN  25
#define FREQUENCY_CORE  1000

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS_MAIN (NSEC_PER_SEC * 1.0 / FREQUENCY_MAIN)
#define PERIOD_NS_CORE (NSEC_PER_SEC * 1.0 / FREQUENCY_CORE)

namespace Aris
{
    namespace RT_CONTROL
    {
        enum EServoState
        {
            EMSTAT_NONE=0,
            EMSTAT_POWEREDOFF=1,
            EMSTAT_STOPPED=2,
            EMSTAT_ENABLED=3,
            EMSTAT_RUNNING=4,
            EMSTAT_HOMING=5,
            EMSTAT_FAULT,
            EMSTAT_INVALID,
            EMSTAT_EMERGE,
        };

        const int N_MOTOR_STATE=9;

        enum EOperationMode
        {
            OM_INVALID     = -1,

            OM_CYCLICPOS  = 8,
            OM_CYCLICVEL  = 9,
            OM_CYCLICTORQ = 10,
        };

        enum EOperationModeDisplay
        {
            OMD_INVALID     = -1,
            OMD_PROFILEPOS = 1,
            OMD_HOMING =6,
            OMD_CYCLICPOS  = 8,
            OMD_CYCLICVEL  = 9,
            OMD_CYCLICTORQ = 10,
            OMD_OTHER      = 18
        };

        enum EServoCommand
        {
            EMCMD_NONE=0,
            EMCMD_POWEROFF=1,
            EMCMD_STOP=2,
            EMCMD_ENABLE=3,
            EMCMD_RUNNING=4,
            EMCMD_GOHOME=5,
        };

        const int N_MOTOR_CMD=6;

        enum EControl_Msg
        {
            CM_PUSH_CMD_TO_MOTORS=100,
            CM_GO_TRAJ,
            CM_CUS_MESSAGE,
            CM_NONE,
        };



        enum EMachineState
        {
            CS_UNINITED,
            CS_INITED,
            CS_COMM_INITED,
            CS_RTTASK_STARTED,
            CS_STOPPED,
        };

        /*
        * CSysInitParameters should be used by SetSysInitializer
        */
        struct CSysInitParameters
        {
        public:
            int motorNum{ACTUAL_MOTOR_NUMBER};
            int homeMode{17};
            int homeAccel{25600};
            int homeLowSpeed{256};
            int homeHighSpeed{2560};
            int p2pMaxSpeed{2560};
            int p2pSpeed{1792};
            int nsPerCyclePeriod{(int)PERIOD_NS_CORE};
            int homeTorqueLimit{1000};
            int* homeOffsets{nullptr};
            int* driverIDs{nullptr};
        };

        struct CMotorData
        {
        public:
            int Position;
            int Velocity;
            int Torque;
        };

        /*
         * XYL:
         * Add model calculation reslut to CMachineData
         * The order of the data is arranged as described in the model
         *
         * Modify do not modify this class frequently, or the data process will not
        */

        class CAbsModelCalcData
        {
        public:
            double Acceleration[AXIS_NUMBER];
            double Velocity[AXIS_NUMBER];
            double Position[AXIS_NUMBER];

            double PrismActualForce[AXIS_NUMBER];// calculated from the torque
            double PrismDynamicForce[AXIS_NUMBER];// calculated by fast dyn
            double ExternForce[6][3];// six leg extern force

            double IMUAngle[3];
            double IMUVel[3];
            double IMUAcc[3];


        };

        struct CMachineData
        {
        public:
            int motorNum;
            EMachineState machinestate;
            long long int time;
            //Motor Data
            bool isMotorHomed[AXIS_NUMBER];
            EServoState motorsStates[AXIS_NUMBER];
            EServoCommand motorsCommands[AXIS_NUMBER];
            EOperationMode motorsModes[AXIS_NUMBER];
            EOperationModeDisplay motorsModesDisplay[AXIS_NUMBER];
            CMotorData feedbackData[AXIS_NUMBER];//currentFeedback,collected after read()
            CMotorData commandData[AXIS_NUMBER];//lastCommand,collected before write()

            //XYL extended for test
            CAbsModelCalcData absData;
            bool IsLogging=false;
            double IMURawAngle[3];
            double IMURawVel[3];
            double IMURawAcc[3];

        };


        /*
         * RT_MSG related settings
         */
        #define RT_MSG_BUFFER_SIZE 8192
        #define RT_MSG_HEADER_LENGTH sizeof(Aris::Core::MSG_HEADER)
        #define PRINT_INFO_BUFFER_SIZE 200

    }

}
#endif // COMPATIBLE_H
