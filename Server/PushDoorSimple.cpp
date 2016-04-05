#include "PushDoorSimple.h"
#include "rtdk.h"
#include <iostream>
#include <cmath>
#include <cstring>

const double PushDoorSimplePlanner::rLegs[2][6]=
    {{0.65, 0.00, -0.65, -0.65,  0.00,  0.65},
     {0.30, 0.45,  0.30, -0.30, -0.45, -0.30}};

const int PushDoorSimplePlanner::LEG_MAP[6] =
    {0, 1, 2, 5, 4, 3};

const double PushDoorSimplePlanner::DISTANCE_COM_TO_FSR = 1.0;

PushDoorSimplePlanner::PushDoorSimplePlanner()
{}

PushDoorSimplePlanner::~PushDoorSimplePlanner()
{}

int PushDoorSimplePlanner::Initialize()
{
    gaitState = VGS_READY;

    InitParams();

    InitStates();
    
    is_ASP_BSW = true;
    
    return 0;
}

void PushDoorSimplePlanner::InitParams()
{
    dt             = 0.001;
    mRobot         = 40;
    IRobot         = 100;
    forceSafe      = 40;
    vMax           = 0.11;
    bVirtual       = forceSafe / vMax;
    l0             = 2.5;
    kCorrect       = forceSafe / l0;
    kAngle         = 10;
    bAngle         = 40;
    rStable        = 0.27;
    heightCOM      = 0.85;
    mActual        = 270;
    heightStep     = 0.04;
    THalfStep      = 2.2;
    tauFoothold    = 0.06;
    Tshift         = 3;
    LenShift       = -0.1;
    longStepLength = -0.36;
    velDetect      = -0.04;
    TDetect        = 4;
    desiredOffset  = -0.08;

    robotFrontier  = -0.77;
    xStage1        = longStepLength + desiredOffset + robotFrontier;
    xStage2        = longStepLength + desiredOffset + robotFrontier + 0.1;
}

int PushDoorSimplePlanner::Start(double timeNow)
{
    if ( gaitState == VGS_READY )
    {
        startTime = timeNow;
        lastTDTime = 0;
        gaitState = VGS_LSTEP_FORWARD;
    }
    return 0;
}
    
int PushDoorSimplePlanner::RequireStop(double timeNow)
{
    gaitState = VGS_STOPPED;
    return 0;
}

int PushDoorSimplePlanner::DoIteration(
        double timeNow,
        double* fext,
        double* legPositionList)
{
    if (gaitState == VGS_READY)
    {
        // output the initial position
        // actually do nothing
    }
    else if (gaitState == VGS_LSTEP_FORWARD)
    {
        double timeFromStart = timeNow - startTime;
        double pivot, tmp1, tmp2;
        if (timeFromStart > (THalfStep*2))
        {
            gaitState = VGS_DETECT_EDGE;
            lastTDTime = 0;
            startTime = timeNow;
            is_ASP_BSW = !is_ASP_BSW; // switch the gait state
            // reserve the foothold position of the last half-step
            memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
            memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
            memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));
        }
        else
        {
            // Body traj
            GetPivot(timeFromStart / (THalfStep*2), pivot, tmp1, tmp2);
            svRobot[0] = pivot/3.1416*longStepLength;
            svRobot[1] = 0; 
            svRobot[2] = 0; 
            // Planning for the first half step
            // Update gait state
            if (timeFromStart - lastTDTime > THalfStep) // A half-step is completed
            {
                rt_printf("Second Step----------------\n");
                is_ASP_BSW = !is_ASP_BSW; // switch the gait state
                // reserve the foothold position of the last half-step
                lastTDTime = timeFromStart;
                memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
                memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
                memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));
            }

            // Calculate the next footholds of the swinging legs 

            if (is_ASP_BSW) // B legs are swinging
            {
                svFootholdDot[2] = 0; 
                svFoothold[2] = longStepLength; 
                svFootholdDot[3] = 0; 
                svFoothold[3] = 0; 

                svFootholdDirDot[1] = 0; 
                svFootholdDir[1] = 0; 
            }
            else // A legs are swinging
            {
                svFootholdDot[0] = 0; 
                svFoothold[0] = longStepLength;
                svFootholdDot[1] = 0; 
                svFoothold[1] = 0; 

                svFootholdDirDot[0] = 0; 
                svFootholdDir[0] = 0; 
            }
            // Calculate footholds for each leg
            CalculateFootholdOfEachLeg(svFoothold, svFootholdDir, HFoothold);

            // Leg motion planning
            double timeRatio = (timeFromStart - lastTDTime) / THalfStep;
            LegMotionPlanning(timeRatio, HFoothold, lastHFoothold, svLeg);
        }
        
    }
    else if (gaitState == VGS_DETECT_EDGE)
    {
        rt_printf("DETECTING EDGE\n");
        double timeFromStart = timeNow - startTime;
        if (timeNow - startTime > TDetect)
        {
            startTime = timeNow;
            gaitState = VGS_STOPPED;
            lastTDTime = 0;
        }
        else if(fabs(fext[0]) > 20)
        {
            startTime = timeNow;
            detectedOffset = timeFromStart * velDetect;
            gaitState = VGS_ADJUST;
            lastTDTime = 0;
            xStage1 = svRobot[0] + robotFrontier;
            xStage2 = svRobot[0] + robotFrontier + 0.1;
            
            rt_printf("GO ADJUST: %f\n %f\n", xStage1, xStage2);
        }
        else
        {
            svRobot[0] = timeFromStart * velDetect + longStepLength; // Move backward a little
            svRobot[1] = 0;
            svRobot[2] = 0;
        }
    }
    else if (gaitState == VGS_ADJUST)
    {
        double timeFromStart = timeNow - startTime;
        double pivot, tmp1, tmp2;
        if (timeFromStart > (THalfStep*2))
        {
            gaitState = VGS_PUSH_DOOR;
            pushCount = 0;
            lastTDTime = 0;
            startTime = timeNow;
            is_ASP_BSW = !is_ASP_BSW; // switch the gait state
            // reserve the foothold position of the last half-step
            memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
            memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
            memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));
            CopyStates(lastEndingSvRobot, svRobot, 6);
            CopyStates(lastEndingSvFthd, svFoothold, 4);
            rt_printf("GO PUSH\n");
            for(int i = 0; i < 3; i++)
            {
                rt_printf("BODY: %f\n", svRobot[i]);
            }
            for(int i = 0; i < 18; i++)
            {
                rt_printf("LEG: %f\n", svLeg[i]);
            }
            rt_printf("----------------------\n");
        }
        else
        {
            // Planning for the first half step
            // Update gait state
            if (timeFromStart - lastTDTime > THalfStep) // A half-step is completed
            {
                is_ASP_BSW = !is_ASP_BSW; // switch the gait state
                // reserve the foothold position of the last half-step
                lastTDTime = timeFromStart;
                memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
                memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
                memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));
            }

            // Calculate the next footholds of the swinging legs 

            if (is_ASP_BSW) // B legs are swinging
            {
                svFootholdDot[2] = 0; 
                svFoothold[2] = longStepLength + (detectedOffset - desiredOffset); 
                svFootholdDot[3] = 0; 
                svFoothold[3] = 0; 

                svFootholdDirDot[1] = 0; 
                svFootholdDir[1] = 0; 
            }
            else // A legs are swinging
            {
                svFootholdDot[0] = 0; 
                svFoothold[0] = longStepLength + (detectedOffset - desiredOffset); 
                svFootholdDot[1] = 0; 
                svFoothold[1] = 0; 

                svFootholdDirDot[0] = 0; 
                svFootholdDir[0] = 0; 
            }
            // Calculate footholds for each leg
            CalculateFootholdOfEachLeg(svFoothold, svFootholdDir, HFoothold);

            // Leg motion planning
            double timeRatio = (timeFromStart - lastTDTime) / THalfStep;
            LegMotionPlanning(timeRatio, HFoothold, lastHFoothold, svLeg);
        }
    }
    else if (gaitState == VGS_PUSH_DOOR)
    {
        double timeFromStart = timeNow - startTime;
        double pivot, tmp1, tmp2;
        if (pushCount > 8)
        {
            gaitState = VGS_STOPPED;
        }
        else if (timeFromStart > (THalfStep*2))
        {
            pushCount++;
            gaitState = VGS_PUSH_DOOR;
            CopyStates(lastEndingSvRobot, svRobot, 6);
            CopyStates(lastEndingSvFthd, svFoothold, 4);
            lastTDTime = 0;
            startTime = timeNow;
            is_ASP_BSW = !is_ASP_BSW; // switch the gait state
            // reserve the foothold position of the last half-step
            memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
            memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
            memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));
            rt_printf("GO PUSH AGAIN\n");
        }
        else
        {
            // Body traj
            GetPivot(timeFromStart / (THalfStep*2), pivot, tmp1, tmp2);
            svRobot[0] = lastEndingSvRobot[0] + pivot/3.1416 * longStepLength;
            svRobot[1] = 0; 
            svRobot[2] = 0; 
            // Planning for the first half step
            // Update gait state
            if (timeFromStart - lastTDTime > THalfStep) // A half-step is completed
            {
                is_ASP_BSW = !is_ASP_BSW; // switch the gait state
                // reserve the foothold position of the last half-step
                lastTDTime = timeFromStart;
                memcpy(lastsvFoothold, svFoothold, sizeof(svFoothold));
                memcpy(lastsvFootholdDir, svFootholdDir, sizeof(svFootholdDir));
                memcpy(lastHFoothold, HFoothold, sizeof(HFoothold));
            }

            // Calculate the next footholds of the swinging legs 

            if (is_ASP_BSW) // B legs are swinging
            {
                svFootholdDot[2] = 0; 
                svFoothold[2] = lastEndingSvFthd[2] + longStepLength; 
                svFootholdDot[3] = 0; 
                svFoothold[3] = 0; 

                svFootholdDirDot[1] = 0; 
                svFootholdDir[1] = 0; 
            }
            else // A legs are swinging
            {
                svFootholdDot[0] = 0; 
                svFoothold[0] = lastEndingSvFthd[0] + longStepLength;
                svFootholdDot[1] = 0; 
                svFoothold[1] = 0; 

                svFootholdDirDot[0] = 0; 
                svFootholdDir[0] = 0; 
            }
            // Calculate footholds for each leg
            CalculateFootholdOfEachLeg(svFoothold, svFootholdDir, HFoothold);

            // Leg motion planning
            double timeRatio = (timeFromStart - lastTDTime) / THalfStep;
            LegMotionPlanningWithHeight(timeRatio, HFoothold, lastHFoothold, svLeg);

            //for(int i = 0; i < 3; i++)
            //{
                //rt_printf("BODY: %f\n", svRobot[i]);
            //}
            //for(int i = 0; i < 18; i++)
            //{
                //rt_printf("LEG: %f\n", svLeg[i]);
            //}
        }
        
        // Print to debug
    }
    else
    {
        // do nothing, hold where it is
    }
    // implictly use the svLeg and bodyOffset
    MapLegPosToActual(legPositionList);

    return 0;
}

void PushDoorSimplePlanner::GetPivot(const double timeRatio, double& rawPivot, double& lenPivot, double& heightPivot)
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
    rawPivot = pivot;
}

template<std::size_t row, std::size_t col>
void PushDoorSimplePlanner::LegMotionPlanning(const double timeRatio, double (&HFoothold)[row][col], double (&lastHFoothold)[row][col], double* svLeg)
{
    double lenPivot = 0;
    double heightPivot = 0;
    double rawPivot;
    GetPivot(timeRatio, rawPivot, lenPivot, heightPivot);
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
void PushDoorSimplePlanner::LegMotionPlanningWithHeight(const double timeRatio, double (&HFoothold)[row][col], double (&lastHFoothold)[row][col], double* svLeg)
{
    double lenPivot = 0;
    double heightPivot = 0;
    double rawPivot;
    GetPivot(timeRatio, rawPivot, lenPivot, heightPivot);
    if (is_ASP_BSW)
    {
        for (int i = 0; i < 3; ++i)
        {
            double terrainHeightStance    = GetTerrainHeightData(HFoothold[0][i*2], HFoothold[1][i*2]);
            double terrainHeightSwingNext = GetTerrainHeightData(HFoothold[0][i*2+1], HFoothold[1][i*2+1]);
            double terrainHeightSwingLast = GetTerrainHeightData(lastHFoothold[0][i*2+1], lastHFoothold[1][i*2+1]);
            svLeg[(i*2)*3 + 0] = HFoothold[0][i*2];
            svLeg[(i*2)*3 + 1] = HFoothold[1][i*2];
            svLeg[(i*2)*3 + 2] = terrainHeightStance; 
            
            svLeg[(i*2+1)*3 + 0] = lastHFoothold[0][i*2+1] + 
                                   lenPivot * (HFoothold[0][i*2+1] - lastHFoothold[0][i*2+1]);
            svLeg[(i*2+1)*3 + 1] = lastHFoothold[1][i*2+1] + 
                                   lenPivot * (HFoothold[1][i*2+1] - lastHFoothold[1][i*2+1]);
            svLeg[(i*2+1)*3 + 2] = terrainHeightSwingLast + 
                                   lenPivot * (terrainHeightSwingNext - terrainHeightSwingLast) + 
                                   heightPivot; 
        }
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            double terrainHeightStance    = GetTerrainHeightData(HFoothold[0][i*2+1], HFoothold[1][i*2+1]);
            double terrainHeightSwingNext = GetTerrainHeightData(HFoothold[0][i*2], HFoothold[1][i*2]);
            double terrainHeightSwingLast = GetTerrainHeightData(lastHFoothold[0][i*2], lastHFoothold[1][i*2]);
            svLeg[(i*2+1)*3 + 0] = HFoothold[0][i*2+1];
            svLeg[(i*2+1)*3 + 1] = HFoothold[1][i*2+1];
            svLeg[(i*2+1)*3 + 2] = terrainHeightStance; 
            
            svLeg[(i*2)*3 + 0] = lastHFoothold[0][i*2] + 
                                   lenPivot * (HFoothold[0][i*2] - lastHFoothold[0][i*2]);
            svLeg[(i*2)*3 + 1] = lastHFoothold[1][i*2] + 
                                   lenPivot * (HFoothold[1][i*2] - lastHFoothold[1][i*2]);
            svLeg[(i*2)*3 + 2] = terrainHeightSwingLast + 
                                 lenPivot * (terrainHeightSwingNext - terrainHeightSwingLast) +
                                 heightPivot; 
        }
    }
}

template<std::size_t row, std::size_t col>
void PushDoorSimplePlanner::CalculateFootholdOfEachLeg(
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

void PushDoorSimplePlanner::EstimateNextFootholdCenter(
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

void PushDoorSimplePlanner::UpdateRobotBodyState(
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
    
void PushDoorSimplePlanner::GetVirtualForce(
        const double timeFromStart, 
        double* svRobot, 
        double* posTracking, 
        double* fVirtualG,
        VIRTUAL_GAIT_STATE gaitState)
{
    double fTangent[2] = {-forceSafe * cos(posTracking[2]), 
                          -forceSafe * sin(posTracking[2])};

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
    double fDamp[2] = {-bVirtual * svRobot[3],
                       -bVirtual * svRobot[4]};

    double angleError = svRobot[2] - posTracking[2];

    double trqVirtual = -kAngle * angleError - bAngle * (angleError - lastAngleError) / dt;

    fVirtualG[0] = fPush[0] + fDamp[0];
    fVirtualG[1] = fPush[1] + fDamp[1];
    fVirtualG[2] = trqVirtual;
    lastAngleError = angleError;
}

void PushDoorSimplePlanner::GetForceInGlobalCoordinate(double* fSensor, double* svRobot, double* fContact)
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

void PushDoorSimplePlanner::GetTrackingPoint(double* posRobot, double* posTracking)
{
    // Ref path: 0*x + 1*y = 0
    posTracking[0] = posRobot[0];
    posTracking[1] = 0;
    posTracking[2] = 0;
}

void PushDoorSimplePlanner::ClearStates(double* stateVector, int length)
{
    for(int i = 0; i < length; i++)
    {
        stateVector[i] = 0;
    }
}

void PushDoorSimplePlanner::InitStates()
{
    using namespace std;
    ClearStates(svRobot, 6);
    ClearStates(posTracking, 3);
    ClearStates(svEstFoothold, 3);
    ClearStates(bodyOffset, 3);
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

void PushDoorSimplePlanner::MapLegPosToActual(double* legPositionList)
{
    double thetaBody = svRobot[2];
    double svLegLocal[18];
    double svLegWithBodyOffset[18];
    double tmp1, tmp2;

    // firstly transform the leg positions to local model COM coordinates
    for(int i = 0; i < 6; i++)
    {
        tmp1 = svLeg[i*3 + 0] - svRobot[0];
        tmp2 = svLeg[i*3 + 1] - svRobot[1];
        svLegLocal[i*3 + 0] =  cos(thetaBody) * tmp1 + sin(thetaBody) * tmp2;
        svLegLocal[i*3 + 1] = -sin(thetaBody) * tmp1 + cos(thetaBody) * tmp2;
        svLegLocal[i*3 + 2] = svLeg[i*3 + 2] - heightCOM;
    }

    // secondly transform the leg positions to local actual body COM coordinates considering the body offset
    for(int i = 0; i < 6; i++)
    {
        tmp1 = svLegLocal[i*3 + 0] - bodyOffset[0];
        tmp2 = svLegLocal[i*3 + 1] - bodyOffset[1];

        svLegWithBodyOffset[i*3 + 0] =  cos(bodyOffset[2]) * tmp1 + sin(bodyOffset[2]) * tmp2;
        svLegWithBodyOffset[i*3 + 1] = -sin(bodyOffset[2]) * tmp1 + cos(bodyOffset[2]) * tmp2;
        svLegWithBodyOffset[i*3 + 2] = svLegLocal[i*3 + 2];
    }
    // Then map the local leg position to actual leg coordinates
    for(int i = 0; i < 6; i++)
    {
        legPositionList[i*3 + 0] = -svLegWithBodyOffset[LEG_MAP[i]*3 + 1];
        legPositionList[i*3 + 1] =  svLegWithBodyOffset[LEG_MAP[i]*3 + 2];
        legPositionList[i*3 + 2] = -svLegWithBodyOffset[LEG_MAP[i]*3 + 0];
    }
}

PushDoorSimplePlanner::InternalData PushDoorSimplePlanner::GetInternalData()
{
    CopyStates(internalData.svRobotD, svRobot, 6); 
    CopyStates(internalData.svFthdD, svFoothold, 4); 
    CopyStates(internalData.svFthdDirD, svFootholdDir, 2); 
    CopyStates(internalData.svLegD, svLeg, 18); 
    CopyStates(internalData.fContactGD, fContactG, 3); 

    return internalData;
}

template<class U, class T>
void PushDoorSimplePlanner::CopyStates(U* stateVectorDest, T* stateVectorSrc, int length)
{
    for(int i = 0; i < length; i++)
    {
        stateVectorDest[i] = stateVectorSrc[i];
    }
}


double PushDoorSimplePlanner::GetTerrainHeightData(double x, double y)
{
    if (x < xStage1)
    {
        return 0.0;
    }
    else if ( x < xStage2 )
    {
        return 0.0;
    }
    return 0;
}
