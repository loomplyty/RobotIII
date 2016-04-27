#include"Line.h"
#include<cmath>

CSmoothParameter::CSmoothParameter()
{

}

CSmoothParameter::~CSmoothParameter()
{

}


void CSmoothParameter::Init(double paraInit, double paraFinal)
{
    this->ParameterInitialValue=paraInit;
    this->ParameterFinalValue=paraFinal;
    this->ParameterSpan=this->ParameterFinalValue-this->ParameterInitialValue;

//    std::cout<<this->ParameterInitialValue<<std::endl;
//    std::cout<<this->ParameterFinalValue<<std::endl;
//    std::cout<<this->ParameterSpan<<std::endl;
}

void CSmoothParameter::MapTimeToParam(double t_norm, double &t_param)
{

    double t=t_norm;
    // this will make sure the t will not go out of the range
    if(t<0.0)
    {
        t=0.0;
    }
    else if(t>1.0)
    {
        t=1.0;
    }

    t_param=(t*SmoothSpan-sin(t*SmoothSpan))*ParameterSpan/SmoothSpan+ParameterInitialValue;
    ParameterCurrent=t_param;
}

/************CSemiCycleExt******************************************************************/

CSemiCycleExt::CSemiCycleExt(){}

CSemiCycleExt::~CSemiCycleExt(){}


void CSemiCycleExt::SetCurrentParam(double param)
{
    CurrentParam=param;

    if(IsClockwise)// start is big
    {
        if(param<ParamNearStart&&param>ParamNearEnd)
        {
            CurrentX=CenterX+Radius*cos(param)-StartX;
            CurrentY=CenterY+Radius*sin(param)-StartY;
        }
        else
        {
            if(param>ParamNearStart-0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+Radius*cos(param)-StartX;
                    CurrentY=CenterY+Radius*sin(ParamNearStart)-StartY;

                    break;
                case 'y':
                    CurrentX=CenterX+Radius*cos(ParamNearStart)-StartX;


//                    printf("%f %f %f %f\n",
//                           StartX,
//                           CurrentX,
//                           CenterX,
//                           Radius*cos(SwitchParamNearStart));

                    CurrentY=CenterY+Radius*sin(param)-StartY;

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+Radius*sin(ParamNearEnd)-StartY;
                    CurrentX=CenterX+Radius*cos(param)-StartX;

                    break;
                case 'y':
                    CurrentY=CenterY+Radius*sin(param)-StartY;
                    CurrentX=CenterX+Radius*cos(ParamNearEnd)-StartX;
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }

        }
    }
    else
    {
        if(param>ParamNearStart&&param<ParamNearEnd)
        {
            CurrentX=CenterX+Radius*cos(param)-StartX;
            CurrentY=CenterY+Radius*sin(param)-StartY;
        }
        else
        {
            if(param<ParamNearStart+0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+Radius*cos(param)-StartX;
                    CurrentY=CenterY+Radius*sin(ParamNearStart)-StartY;

                    break;
                case 'y':
                    CurrentX=CenterX+Radius*cos(ParamNearStart)-StartX;
                    CurrentY=CenterY+Radius*sin(param)-StartY;

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+Radius*sin(ParamNearEnd)-StartY;
                    CurrentX=CenterX+Radius*cos(param)-StartX;

                    break;
                case 'y':
                    CurrentY=CenterY+Radius*sin(param)-StartY;
                    CurrentX=CenterX+Radius*cos(ParamNearEnd)-StartX;
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }


        }

    }


}


void CSemiCycleExt::SetCycleParam(
        double center_x,
        double center_y,
        double radius,
        double start_param,
        bool isClockwise)
{
    CenterX=center_x;
    CenterY=center_y;
    Radius=radius;
    IsClockwise=isClockwise;
    SetStartParam(start_param);
}

void CSemiCycleExt::SetStartParam(double param)
{
    StartParam=param;
//    StartX=CenterX+Radius*cos(param);
//    StartY=CenterY+Radius*sin(param);

}

void CSemiCycleExt::SetSwitchPoint(
        double switch_param_start,
        char switch_direction_start,
        double switch_param_end,
        char switch_direction_end)
{
    ParamNearStart=switch_param_start;
    DirectionNearStart=switch_direction_start;
    ParamNearEnd=switch_param_end;
    DirectionNearEnd=switch_direction_end;


    double param=StartParam;

    if(IsClockwise)// start is big
    {
        if(param<ParamNearStart&&param>ParamNearEnd)
        {
            CurrentX=CenterX+Radius*cos(param);
            CurrentY=CenterY+Radius*sin(param);
        }
        else
        {
            if(param>ParamNearStart-0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+Radius*cos(param);
                    CurrentY=CenterY+Radius*sin(ParamNearStart);

//                    printf("%f %f %f %f\n",
//                           StartX,
//                           CurrentX,
//                           CenterX,
//                           Radius*cos(SwitchParamNearStart));
//                    printf("%f %f %f\n",param,SwitchParamNearStart,SwitchParamNearEnd);

                    break;
                case 'y':
                    CurrentX=CenterX+Radius*cos(ParamNearStart);
                    CurrentY=CenterY+Radius*sin(param);

//                    printf("%f %f %f %f\n",
//                           StartX,
//                           CurrentX,
//                           CenterX,
//                           Radius*cos(SwitchParamNearStart));
//                    printf("%f %f %f\n",param,SwitchParamNearStart,SwitchParamNearEnd);

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+Radius*sin(ParamNearEnd);
                    CurrentX=CenterX+Radius*cos(param);



                    break;
                case 'y':
                    CurrentY=CenterY+Radius*sin(param);
                    CurrentX=CenterX+Radius*cos(ParamNearEnd);

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }

        }
    }
    else
    {
        if(param>ParamNearStart&&param<ParamNearEnd)
        {
            CurrentX=CenterX+Radius*cos(param);
            CurrentY=CenterY+Radius*sin(param);
        }
        else
        {
            if(param<DirectionNearStart+0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+Radius*cos(param);
                    CurrentY=CenterY+Radius*sin(ParamNearStart);

                    break;
                case 'y':
                    CurrentX=CenterX+Radius*cos(ParamNearStart);
                    CurrentY=CenterY+Radius*sin(param);
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+Radius*sin(ParamNearEnd);
                    CurrentX=CenterX+Radius*cos(param);

                    break;
                case 'y':
                    CurrentY=CenterY+Radius*sin(param);
                    CurrentX=CenterX+Radius*cos(ParamNearEnd);
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }


        }

    }
    StartX=CurrentX;
    StartY=CurrentY;
//    std::cout<<SwitchParamNearStart<<"\t"<<SwitchParamNearEnd <<std::endl;
//    std::cout<<StartX<<"\t"<<StartY<<std::endl;
//    std::cout<<CenterX<<"\t"<<CenterY<<std::endl;


}


void CSemiCycleExt::GetCurvePoint(double &x, double &y)
{
    x=CurrentX;
    y=CurrentY;
}

void CSemiCycleExt::SetReverseDirection()
{
    if(IsClockwise)
    {
        IsClockwise=false;
    }
    else
    {
        IsClockwise=true;
    }
}

double CSemiCycleExt::GetCurrentParam()
{
    return this->CurrentParam;
}

/**************************CSemiEllipseExt******************************************************/

CSemiEllipseExt::CSemiEllipseExt(){}

CSemiEllipseExt::~CSemiEllipseExt(){}



void CSemiEllipseExt::SetCurrentParam(double param)
{
    CurrentParam=param;

    if(IsClockwise)// start is big
    {
        if(param<ParamNearStart&&param>ParamNearEnd)
        {
            CurrentX=CenterX+RadiusA*cos(param)-StartX;
            CurrentY=CenterY+RadiusB*sin(param)-StartY;
        }
        else
        {
            if(param>ParamNearStart-0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+RadiusA*cos(param)-StartX;
                    CurrentY=CenterY+RadiusB*sin(ParamNearStart)-StartY;

                    break;
                case 'y':
                    CurrentX=CenterX+RadiusA*cos(ParamNearStart)-StartX;


//                    printf("%f %f %f %f\n",
//                           StartX,
//                           CurrentX,
//                           CenterX,
//                           Radius*cos(SwitchParamNearStart));

                    CurrentY=CenterY+RadiusB*sin(param)-StartY;

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+RadiusB*sin(ParamNearEnd)-StartY;
                    CurrentX=CenterX+RadiusA*cos(param)-StartX;

                    break;
                case 'y':
                    CurrentY=CenterY+RadiusB*sin(param)-StartY;
                    CurrentX=CenterX+RadiusA*cos(ParamNearEnd)-StartX;
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }

        }
    }
    else
    {
        if(param>ParamNearStart&&param<ParamNearEnd)
        {
            CurrentX=CenterX+RadiusA*cos(param)-StartX;
            CurrentY=CenterY+RadiusB*sin(param)-StartY;
        }
        else
        {
            if(param<ParamNearStart+0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+RadiusA*cos(param)-StartX;
                    CurrentY=CenterY+RadiusB*sin(ParamNearStart)-StartY;

                    break;
                case 'y':
                    CurrentX=CenterX+RadiusA*cos(ParamNearStart)-StartX;
                    CurrentY=CenterY+RadiusB*sin(param)-StartY;

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+RadiusB*sin(ParamNearEnd)-StartY;
                    CurrentX=CenterX+RadiusA*cos(param)-StartX;

                    break;
                case 'y':
                    CurrentY=CenterY+RadiusB*sin(param)-StartY;
                    CurrentX=CenterX+RadiusA*cos(ParamNearEnd)-StartX;
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }


        }

    }


}

void CSemiEllipseExt::SetCycleParam(
        double center_x,
        double center_y,
        double radiusA,
        double radiusB,
        double start_param,
        bool isClockwise)
{
    CenterX=center_x;
    CenterY=center_y;
    RadiusA=radiusA;
    RadiusB=radiusB;
    IsClockwise=isClockwise;
    SetStartParam(start_param);
}

void CSemiEllipseExt::SetStartParam(double param)
{
    StartParam=param;

}

void CSemiEllipseExt::SetSwitchPoint(
        double switch_param_start,
        char switch_direction_start,
        double switch_param_end,
        char switch_direction_end)
{
    ParamNearStart=switch_param_start;
    DirectionNearStart=switch_direction_start;
    ParamNearEnd=switch_param_end;
    DirectionNearEnd=switch_direction_end;


    double param=StartParam;

    if(IsClockwise)// start is big
    {
        if(param<ParamNearStart&&param>ParamNearEnd)
        {
            CurrentX=CenterX+RadiusA*cos(param);
            CurrentY=CenterY+RadiusB*sin(param);
        }
        else
        {
            if(param>ParamNearStart-0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+RadiusA*cos(param);
                    CurrentY=CenterY+RadiusB*sin(ParamNearStart);


//                    printf("%f %f %f %f\n",
//                           StartX,
//                           CurrentX,
//                           CenterX,
//                           Radius*cos(SwitchParamNearStart));
//                    printf("%f %f %f\n",param,SwitchParamNearStart,SwitchParamNearEnd);

                    break;
                case 'y':
                    CurrentX=CenterX+RadiusA*cos(ParamNearStart);
                    CurrentY=CenterY+RadiusB*sin(param);

//                    printf("xx  %f %f\n",CurrentX,CurrentY);

//                    printf("%f %f %f %f\n",
//                           StartX,
//                           CurrentX,
//                           CenterX,
//                           Radius*cos(SwitchParamNearStart));
//                    printf("%f %f %f\n",param,SwitchParamNearStart,SwitchParamNearEnd);

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+RadiusB*sin(ParamNearEnd);
                    CurrentX=CenterX+RadiusA*cos(param);



                    break;
                case 'y':
                    CurrentY=CenterY+RadiusB*sin(param);
                    CurrentX=CenterX+RadiusA*cos(ParamNearEnd);

                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }

        }
    }
    else
    {
        if(param>ParamNearStart&&param<ParamNearEnd)
        {
            CurrentX=CenterX+RadiusA*cos(param);
            CurrentY=CenterY+RadiusB*sin(param);
        }
        else
        {
            if(param<DirectionNearStart+0.001)
            {
                switch(DirectionNearStart)
                {
                case 'x':
                    CurrentX=CenterX+RadiusA*cos(param);
                    CurrentY=CenterY+RadiusB*sin(ParamNearStart);

                    break;
                case 'y':
                    CurrentX=CenterX+RadiusA*cos(ParamNearStart);
                    CurrentY=CenterY+RadiusB*sin(param);
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }

            }
            else
            {
                switch(DirectionNearEnd)
                {
                case 'x':
                    CurrentY=CenterY+RadiusB*sin(ParamNearEnd);
                    CurrentX=CenterX+RadiusA*cos(param);

                    break;
                case 'y':
                    CurrentY=CenterY+RadiusB*sin(param);
                    CurrentX=CenterX+RadiusA*cos(ParamNearEnd);
                    break;
                default:
    //                CurrentX=CenterX+Radius*cos(param)-StartX;
    //                CurrentY=CenterY+Radius*sin(param)-StartY;
                    break;
                }
            }


        }

    }
    StartX=CurrentX;
    StartY=CurrentY;
//    std::cout<<SwitchParamNearStart<<"\t"<<SwitchParamNearEnd <<std::endl;
//    std::cout<<StartX<<"\t"<<StartY<<std::endl;
//    std::cout<<CenterX<<"\t"<<CenterY<<std::endl;


}

void CSemiEllipseExt::GetCurvePoint(double &x, double &y)
{
    x=CurrentX;
    y=CurrentY;
}

void CSemiEllipseExt::SetReverseDirection()
{
    if(IsClockwise)
    {
        IsClockwise=false;
    }
    else
    {
        IsClockwise=true;
    }
}

double CSemiEllipseExt::GetCurrentParam()
{
    return this->CurrentParam;
}


