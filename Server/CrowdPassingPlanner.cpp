#include "CrowdPassingPlanner.h"
#include <iostream>
#include <cmath>
#include <cstring>

const double CrowdPassingPlanner::rLegs[2][6]=
    {{0.65, 0.00, -0.65, -0.65,  0.00,  0.65},
     {0.30, 0.45,  0.30, -0.30, -0.45, -0.30}};

const int CrowdPassingPlanner::LEG_MAP[6] = 
    {0, 1, 2, 5, 4, 3};

const double CrowdPassingPlanner::DISTANCE_COM_TO_FSR = 1.0;

CrowdPassingPlanner::CrowdPassingPlanner()
{}

CrowdPassingPlanner::~CrowdPassingPlanner()
{}

int CrowdPassingPlanner::Initialize()
{
    gaitState = VGS_READY;

    InitParams();

    InitStates();
    
    is_ASP_BSW = true;
    
    return 0;
}

void CrowdPassingPlanner::InitParams()
{
    dt         = 0.001;
    mRobot     = 60;
    IRobot     = 100;
    forceSafe  = 40;
    vMax       = 0.11;
    bVirtual   = forceSafe / vMax;
    l0         = 2.5;
    kCorrect   = forceSafe / l0;
    kAngle     = 10;
    bAngle     = 40;
    rStable    = 0.27;
    heightCOM  = 0.85;
    mActual    = 270;
    heightStep = 0.04;
    THalfStep  = 1.8;
    tauFoothold = 0.06;
}

int CrowdPassingPlanner::Start(double timeNow)
{
    if ( gaitState == VGS_READY )
    {
        startTime = timeNow;
        lastTDTime = 0;
        gaitState = VGS_STARTED;
    }
    return 0;
}

int CrowdPassingPlanner::RequireStop(double timeNow)
{
    if ( gaitState == VGS_STARTED )
    {
        requireStopTime = timeNow;
        stepLeft = STEP_TO_COMPLETELY_STOP;
        gaitState = VGS_STOPPING;
    }
    if ( gaitState == VGS_READY)
    {
        gaitState = VGS_STOPPED;
    }
    return 0;
}

int CrowdPassingPlanner::DoIteration(
        double timeNow,
        double* fext,
        double* legPositionList)
{
    if (gaitState == VGS_READY)
    {
        // output the initial position
        // actually do nothing
    }
    else if (gaitState == VGS_STARTED || gaitState == VGS_STOPPING)
    {
        double timeFromStart = timeNow - startTime;
        // Finding the tracking point
        GetTrackingPoint(svRobot, posTracking); 
         
        // Transform the force feedback to the global coordinate
        GetForceInGlobalCoordinate(fext, svRobot, fContactG);

        // Construct the virtual forces
        GetVirtualForce(timeFromStart, svRobot, posTracking, fVirtualG, gaitState);

        // Solve virtual dynamics
        UpdateRobotBodyState(svRobot, fVirtualG, fContactG, posZMP);

        // Update gait state
        if (timeFromStart - lastTDTime > THalfStep) // A half-step is completed
        {
            is_ASP_BSW = !is_ASP_BSW; // switch the gait state
            // reserve the foothold position of the last half-step
            lastTDTime = timeFromStart;
            memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
            memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
            memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));

            if (gaitState == VGS_STOPPING)
            {
                stepLeft--;
                if (stepLeft <= 0)
                {
                    gaitState = VGS_STOPPED;
                }
            }
        }

        // Estimate the next footholds of the swinging legs based on prediction
        EstimateNextFootholdCenter(
                timeFromStart, lastTDTime, svRobot, svFoothold, svFootholdDir);

        // Calculate footholds for each leg
        CalculateFootholdOfEachLeg(svFoothold, svFootholdDir, HFoothold);

        // Leg motion planning
        double timeRatio = (timeFromStart - lastTDTime) / THalfStep;
        LegMotionPlanning(timeRatio, HFoothold, lastHFoothold, svLeg);
        
    }
    else
    {
        // do nothing, hold where it is
    }
    MapLegPosToActual(legPositionList);

    return 0;
}

void CrowdPassingPlanner::GetPivot(const double timeRatio, double& lenPivot, double& heightPivot)
{
    double tacc = 0.35;
    double acc = 3.1415926535897931 / tacc / (1 - tacc);
    double pivot = 0;
    if (timeRatio < tacc)
    {
        pivot = 0.5 * (timeRatio*timeRatio) * acc;
    }
    else if (timeRatio < 1-tacc)
    {
        pivot = 0.5 * tacc * tacc * acc + 
            tacc * acc * (timeRatio - tacc);
    }
    else
    {
        pivot = tacc * acc - 1.5 * tacc*tacc * acc + 
                tacc * acc * (timeRatio - 1 + tacc) - 
                0.5 * (timeRatio - 1 + tacc)*(timeRatio - 1 + tacc) * acc;
    }
    lenPivot = 0.5 * (1 - cos(pivot));
    heightPivot = heightStep * sin(pivot);
}

template<std::size_t row, std::size_t col>
void CrowdPassingPlanner::LegMotionPlanning(const double timeRatio, double (&HFoothold)[row][col], double (&lastHFoothold)[row][col], double* svLeg)
{
    double lenPivot = 0;
    double heightPivot = 0;
    GetPivot(timeRatio, lenPivot, heightPivot);
    if (is_ASP_BSW)
    {
        for (int i = 0; i < 3; ++i)
        {
            svLeg[(i*2)*3 + 0] = HFoothold[0][i*2];
            svLeg[(i*2)*3 + 1] = HFoothold[1][i*2];
            svLeg[(i*2)*3 + 2] = 0; 
            
            svLeg[(i*2+1)*3 + 0] = lastHFoothold[0][i*2+1] + 
                                   lenPivot * (HFoothold[0][i*2+1] - lastHFoothold[0][i*2+1]);
            svLeg[(i*2+1)*3 + 1] = lastHFoothold[1][i*2+1] + 
                                   lenPivot * (HFoothold[1][i*2+1] - lastHFoothold[1][i*2+1]);
            svLeg[(i*2+1)*3 + 2] = heightPivot; 
        }
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            svLeg[(i*2+1)*3 + 0] = HFoothold[0][i*2+1];
            svLeg[(i*2+1)*3 + 1] = HFoothold[1][i*2+1];
            svLeg[(i*2+1)*3 + 2] = 0; 
            
            svLeg[(i*2)*3 + 0] = lastHFoothold[0][i*2] + 
                                   lenPivot * (HFoothold[0][i*2] - lastHFoothold[0][i*2]);
            svLeg[(i*2)*3 + 1] = lastHFoothold[1][i*2] + 
                                   lenPivot * (HFoothold[1][i*2] - lastHFoothold[1][i*2]);
            svLeg[(i*2)*3 + 2] = heightPivot; 
        }
    }
}

template<std::size_t row, std::size_t col>
void CrowdPassingPlanner::CalculateFootholdOfEachLeg(
        double* svFoothold, double* svFootholdDir, double (&HFoothold)[row][col])
{
    double thetaA = svFootholdDir[0];
    double thetaB = svFootholdDir[1];

    for(int i = 0; i < 3; i++)
    {
        HFoothold[0][i*2] = svFoothold[0] + cos(thetaA) * rLegs[0][i*2] - sin(thetaA) * rLegs[1][i*2];
        HFoothold[1][i*2] = svFoothold[1] + sin(thetaA) * rLegs[0][i*2] + cos(thetaA) * rLegs[1][i*2];
        
        HFoothold[0][i*2+1] = svFoothold[2] + cos(thetaB) * rLegs[0][i*2+1] - sin(thetaB) * rLegs[1][i*2+1];
        HFoothold[1][i*2+1] = svFoothold[3] + sin(thetaB) * rLegs[0][i*2+1] + cos(thetaB) * rLegs[1][i*2+1];
    }
}

void CrowdPassingPlanner::EstimateNextFootholdCenter(
        const double timeFromStart, const double lastTDTime, 
        double* svRobot, double* svFoothold, double* svFootholdDir)
{
    // Estimate the foothold center
    if (timeFromStart - lastTDTime < 0.7 * THalfStep) // if the time permits the change of foothold
    {
        double timeInStep = timeFromStart - lastTDTime;
        double timeLeft = 0.5*THalfStep + THalfStep - timeInStep;
        
        for (int i = 0; i < 3; ++i)
        {
            svEstFoothold[i] = svRobot[i] + svRobot[i+3]*timeLeft;
        }
    } 

    // Lowpass filter for the foothold target
    if (is_ASP_BSW) // B legs are swinging
    {
        svFootholdDot[2] = 1/tauFoothold*(svEstFoothold[0] - svFoothold[2]);
        svFoothold[2] = svFoothold[2] + dt * svFootholdDot[2];
        svFootholdDot[3] = 1/tauFoothold*(svEstFoothold[1] - svFoothold[3]);
        svFoothold[3] = svFoothold[3] + dt * svFootholdDot[3];

        svFootholdDirDot[1] = 1/tauFoothold*(svEstFoothold[2] - svFootholdDir[1]);
        svFootholdDir[1] = svFootholdDir[1] + dt * svFootholdDirDot[1];
    }
    else // A legs are swinging
    {
        svFootholdDot[0] = 1/tauFoothold*(svEstFoothold[0] - svFoothold[0]);
        svFoothold[0] = svFoothold[0] + dt * svFootholdDot[0];
        svFootholdDot[1] = 1/tauFoothold*(svEstFoothold[1] - svFoothold[1]);
        svFoothold[1] = svFoothold[1] + dt * svFootholdDot[1];

        svFootholdDirDot[0] = 1/tauFoothold*(svEstFoothold[2] - svFootholdDir[0]);
        svFootholdDir[0] = svFootholdDir[0] + dt * svFootholdDirDot[0];
    }
}

void CrowdPassingPlanner::UpdateRobotBodyState(
        double* svRobot, double* fVirtualG, double* fContactG, double* posZMP)
{
    double fAll[3] = {fVirtualG[0] + fContactG[0],
                      fVirtualG[1] + fContactG[1],
                      fVirtualG[2] + fContactG[2]};
    
    // Calculate accelaration of the body
    double ddot[3] = {fAll[0] / mRobot,
                      fAll[1] / mRobot,
                      fAll[2] / IRobot};

    // Calculate ZMP position
    posZMP[0] = svRobot[0] + heightCOM/mActual/9.81 * (-mActual * ddot[0] + fContactG[0]);
    posZMP[1] = svRobot[1] + heightCOM/mActual/9.81 * (-mActual * ddot[1] + fContactG[1]);

    // Calculate new velocity of the body
    for(int i = 0; i < 3; i++)
    {
        svRobot[3 + i] = ddot[i] * dt + svRobot[3 + i];
    }
    // Calculate new position of the body
    for(int i = 0; i < 3; i++)
    {
        svRobot[i] = svRobot[i + 3] * dt + svRobot[i];
    }
}
    
void CrowdPassingPlanner::GetVirtualForce(
        const double timeFromStart, 
        double* svRobot, 
        double* posTracking, 
        double* fVirtualG,
        VIRTUAL_GAIT_STATE gaitState)
{
    double fTangent[2] = {forceSafe * cos(posTracking[2]), 
                          forceSafe * sin(posTracking[2])};

    if (timeFromStart < 1.0)
    {
        fTangent[0] /= 2.0;
        fTangent[1] /= 2.0;
    }

    double fCorrect[2] = {kCorrect * (posTracking[0] - svRobot[0]), 
                          kCorrect * (posTracking[1] - svRobot[1])};
    double fCombine[2] = {fTangent[0] + fCorrect[0],
                          fTangent[1] + fCorrect[1]};
    // Normalize fCombine to constant magnitude
    double normFCombine = sqrt(fCombine[0]*fCombine[0] + fCombine[1]*fCombine[1]);
    double fPush[2] = {forceSafe / normFCombine * fCombine[0],
                       forceSafe / normFCombine * fCombine[1]};
    if (gaitState == VGS_STOPPING)
    {
        fPush[0] = 0;
        fPush[1] = 0;
    }
    double fDamp[2] = {-bVirtual * svRobot[3],
                       -bVirtual * svRobot[4]};

    double angleError = svRobot[2] - posTracking[2];

    double trqVirtual = -kAngle * angleError - bAngle * (angleError - lastAngleError) / dt;

    fVirtualG[0] = fPush[0] + fDamp[0];
    fVirtualG[1] = fPush[1] + fDamp[1];
    fVirtualG[2] = trqVirtual;
    lastAngleError = angleError;
}

void CrowdPassingPlanner::GetForceInGlobalCoordinate(double* fSensor, double* svRobot, double* fContact)
{
    double fContactL[3];

    // Map the force in Local Coordinate
    fContactL[0] = fSensor[0];
    fContactL[1] = fSensor[1];
    fContactL[2] = fSensor[5] + DISTANCE_COM_TO_FSR * fContactL[1];

    // Map the force to Global Coordinate
    double thetaBody = svRobot[2];
    fContactG[0] = cos(thetaBody) * fContactL[0] - sin(thetaBody) * fContactL[1];
    fContactG[1] = sin(thetaBody) * fContactL[0] + cos(thetaBody) * fContactL[1];
    fContactG[2] = fContactL[2];
}

void CrowdPassingPlanner::GetTrackingPoint(double* posRobot, double* posTracking)
{
    // Ref path: 0*x + 1*y = 0
    posTracking[0] = posRobot[0];
    posTracking[1] = 0;
    posTracking[2] = 0;
}

void CrowdPassingPlanner::ClearStates(double* stateVector, int length)
{
    for(int i = 0; i < length; i++)
    {
        stateVector[i] = 0;
    }
}

void CrowdPassingPlanner::InitStates()
{
    using namespace std;
    ClearStates(svRobot, 6);
    ClearStates(posTracking, 3);
    ClearStates(svEstFoothold, 3);
    for(int i = 0; i < 2; i++)
    {
        svFoothold[i*2]   = svRobot[0];
        svFoothold[i*2+1] = svRobot[1];
        svFootholdDir[i]  = svRobot[2];

        svFootholdDot[i*2]     = 0;
        svFootholdDot[i*2 + 1] = 0;
        svFootholdDirDot[i]    = 0;
    }

    double thetaA = svFootholdDir[0];
    double thetaB = svFootholdDir[1];

    for(int i = 0; i < 3; i++)
    {
        HFoothold[0][i*2] = svFoothold[0] + cos(thetaA) * rLegs[0][i*2] - sin(thetaA) * rLegs[1][i*2];
        HFoothold[1][i*2] = svFoothold[1] + sin(thetaA) * rLegs[0][i*2] + cos(thetaA) * rLegs[1][i*2];
        
        HFoothold[0][i*2+1] = svFoothold[2] + cos(thetaB) * rLegs[0][i*2+1] - sin(thetaB) * rLegs[1][i*2+1];
        HFoothold[1][i*2+1] = svFoothold[3] + sin(thetaB) * rLegs[0][i*2+1] + cos(thetaB) * rLegs[1][i*2+1];
    }

    for(int i = 0; i < 6; i++)
    {
        svLeg[i*3 + 0] = HFoothold[0][i];
        svLeg[i*3 + 1] = HFoothold[1][i];
        svLeg[i*3 + 2] = 0;
    }

    lastAngleError = 0;
    memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
    memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
    memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));

}

void CrowdPassingPlanner::MapLegPosToActual(double* legPositionList)
{
    double thetaBody = svRobot[2];
    double svLegLocal[18];
    double tmp1, tmp2;

    // firstly transform the leg positions to local coordinates
    for(int i = 0; i < 6; i++)
    {
        tmp1 = svLeg[i*3 + 0] - svRobot[0];
        tmp2 = svLeg[i*3 + 1] - svRobot[1];
        svLegLocal[i*3 + 0] =  cos(thetaBody) * tmp1 + sin(thetaBody) * tmp2;
        svLegLocal[i*3 + 1] = -sin(thetaBody) * tmp1 + cos(thetaBody) * tmp2;
        svLegLocal[i*3 + 2] = svLeg[i*3 + 2] - heightCOM;
    }

    // Then map the local leg position to actual leg coordinates
    for(int i = 0; i < 6; i++)
    {
        legPositionList[i*3 + 0] = -svLegLocal[LEG_MAP[i]*3 + 1];
        legPositionList[i*3 + 1] =  svLegLocal[LEG_MAP[i]*3 + 2];
        legPositionList[i*3 + 2] = -svLegLocal[LEG_MAP[i]*3 + 0];
    }
}

CrowdPassingPlanner::InternalData CrowdPassingPlanner::GetInternalData()
{
    CopyStates(internalData.svRobotD, svRobot, 6); 
    CopyStates(internalData.svFthdD, svFoothold, 4); 
    CopyStates(internalData.svFthdDirD, svFootholdDir, 2); 
    CopyStates(internalData.svLegD, svLeg, 18); 
    CopyStates(internalData.fContactGD, fContactG, 3); 

    return internalData;
}

template<class U, class T>
void CrowdPassingPlanner::CopyStates(U* stateVectorDest, T* stateVectorSrc, int length)
{
    for(int i = 0; i < length; i++)
    {
        stateVectorDest[i] = stateVectorSrc[i];
    }
}


