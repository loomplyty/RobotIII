#include "interpolation.h"

double Hermite3(double x, double x1, double x2, double y1, double y2, double m1, double m2)
{
    double alpha1=(1+2*(x-x1)/(x2-x1))*std::pow((x-x2)/(x1-x2),2);
    double alpha2=(1+2*(x-x2)/(x1-x2))*std::pow((x-x1)/(x2-x1),2);
    double beta1=(x-x1)*std::pow((x-x2)/(x1-x2),2);
    double beta2=(x-x2)*std::pow((x-x1)/(x2-x1),2);
    double y=alpha1*y1+alpha2*y2+beta1*m1+beta2*m2;
    return y;
}
