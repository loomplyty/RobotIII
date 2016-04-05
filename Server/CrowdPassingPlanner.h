#ifndef CROWD_PASSING_PLANNER_H
#define CROWD_PASSING_PLANNER_H

#include <cstddef>

class CrowdPassingPlanner
{
public:

    class InternalData
    {
        public:
            float svRobotD[6];
            float svFthdD[4];
            float svFthdDirD[2];
            float svLegD[18];
            float fContactGD[3];
    };

    enum VIRTUAL_GAIT_STATE
    {
        VGS_READY    = 1,
        VGS_STARTED  = 2,
        VGS_STOPPING = 3,
        VGS_STOPPED  = 4
    };

    CrowdPassingPlanner();
    ~CrowdPassingPlanner();

    int Initialize();
    int Start(double timeNow);
    int RequireStop(double timeNow);
    int DoIteration(
            /*IN*/double timeNow, 
            /*IN*/double * fext, 
            /*OUT*/ double * legPositionList); 
    
    VIRTUAL_GAIT_STATE GetState() const { return gaitState;};

    InternalData GetInternalData(); 
private:

    static const int STEP_TO_COMPLETELY_STOP = 8;
    static const double rLegs[2][6];
    static const int LEG_MAP[6];
    static const double DISTANCE_COM_TO_FSR;
    
    // Gait planner state machine
    double startTime;
    double lastTDTime;
    double requireStopTime;
    double stepLeft;

    VIRTUAL_GAIT_STATE gaitState = VGS_READY;

    // Virtual model states
    double svRobot[6];
    double svEstFoothold[3];
    double svFootholdDot[4];
    double svFoothold[4];
    double svFootholdDir[2];
    double svFootholdDirDot[2];
    double svLeg[18];
    double HFoothold[2][6];

    double lastAngleError;
    double lastsvFoothold[4];
    double lastsvFootholdDir[2];
    double lastHFoothold[2][6];
    
    double posTracking[3];
    double posZMP[2];

    double fContactG[3];
    double fVirtualG[3];

    InternalData internalData;

    // Virtual model parameters
    double dt;
    double mRobot;
    double IRobot;
    double forceSafe;
    double vMax;
    double bVirtual;
    double l0;
    double kCorrect;
    double kAngle;
    double bAngle;
    double rStable;
    double heightCOM;
    double mActual;
    // Gait parameters
    double heightStep;
    double THalfStep;
    double tauFoothold;
    bool is_ASP_BSW;

    // functions
    void InitStates();
    void InitParams();
    void GetTrackingPoint(double* posRobot, double* posTracking);
    void ClearStates(double* stateVector, int length);

    template<class U, class T>
    void CopyStates(U* stateVectorDest, T* stateVectorSrc, int length);

    void MapLegPosToActual(double* legPositionList);
    void GetForceInGlobalCoordinate(double* fSensor, double* svRobot, double* fContact);
    void GetVirtualForce (
            const double timeFromStart, 
            double* svRobot, 
            double* posTracking, 
            double* fVirtualG, 
            VIRTUAL_GAIT_STATE gaitState );

    void UpdateRobotBodyState(double* svRobot, double* fVirtualG, double* fContactG, double* posZMP);
    void EstimateNextFootholdCenter(
            const double timeFromStart, const double lastTDTime, 
            double* svRobot, double* svFoothold, double* svFootholdDir);

    template<std::size_t row, std::size_t col>
    void CalculateFootholdOfEachLeg(double* svFoothold, double* svFootholdDir, double (&HFoothold)[row][col]);

    template<std::size_t row, std::size_t col>
    void LegMotionPlanning(const double timeRatio, double (&HFoothold)[row][col], double (&lastHFoothold)[row][col], double* svLeg);

    void GetPivot(const double timeRatio, double& lenPivot, double& heightPivot);
};

#endif

