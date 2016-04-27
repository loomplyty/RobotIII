#ifndef LINE_H
#define LINE_H
#include<cmath>
#include<iostream>
//transformed the time, then any curve is smooth.
// this class take normalized time as input time.
class CSmoothParameter
{
public:
    CSmoothParameter();
    ~CSmoothParameter();
    void Init(double paraInit, double paraFinal);
    void MapTimeToParam(double t_norm, double& t_param);


private:
    static constexpr double SmoothSpan=2*M_PI;
public:
    double ParameterInitialValue;
    double ParameterFinalValue;
    double ParameterSpan;
    double ParameterCurrent;

};


// a semicycle then extended with a line
// it will be a member of CGaitPart;
// as an other kind of GaitInfo
// first kind of GaitInfo is just a straight line
class CSemiCycleExt
{
public:
    CSemiCycleExt();
    ~CSemiCycleExt();
    // after switch point, only one direction will continue moving
    void SetSwitchPoint(double switch_param_start,
                        char switch_direction_start,
                        double switch_param_end,
                        char switch_direction_end);
    void SetCycleParam(
            double center_x,
            double center_y,
            double radius,
            double start_param,
            bool isClockwise);
    void SetStartParam(double param);

    void SetCurrentParam(double param);
    void GetCurvePoint(double& x, double& y);// relative to the start point
    void SetReverseDirection();

    double GetCurrentParam();

public:
    char DirectionNearStart=' ';
    double ParamNearStart=0.0;
    char DirectionNearEnd=' ';
    double ParamNearEnd=0.0;
public:
    bool IsClockwise=true;
    double CenterX;
    double CenterY;
    double Radius;
    double StartParam;

    double CurrentParam;

    double StartX;
    double StartY;

    double CurrentX;
    double CurrentY;
};

class CSemiEllipseExt
{
public:
    CSemiEllipseExt();
    ~CSemiEllipseExt();
    // after switch point, only one direction will continue moving
    void SetSwitchPoint(double switch_param_start,
                        char switch_direction_start,
                        double switch_param_end,
                        char switch_direction_end);
    void SetCycleParam(
            double center_x,
            double center_y,
            double radiusA,
            double radiusB,
            double start_param,
            bool isClockwise);
    void SetStartParam(double param);

    void SetCurrentParam(double param);
    void GetCurvePoint(double& x, double& y);// relative to the start point
    void SetReverseDirection();

    double GetCurrentParam();

public:
    char DirectionNearStart=' ';
    double ParamNearStart=0.0;
    char DirectionNearEnd=' ';
    double ParamNearEnd=0.0;
public:
    bool IsClockwise=true;
    double CenterX;
    double CenterY;
    double RadiusA;
    double RadiusB;
    double StartParam;

    double CurrentParam;

    double StartX;
    double StartY;

    double CurrentX;
    double CurrentY;
};

#endif // LINE_H
