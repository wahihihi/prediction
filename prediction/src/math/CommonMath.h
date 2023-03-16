//
// Created by ai on 23-3-16.
//

#ifndef PREDICTION_COMMONMATH_H
#define PREDICTION_COMMONMATH_H
#include <tuple>
#include <math.h>
#include <iostream>


namespace aptiv{
namespace math{

class CommonMath {
public:
    CommonMath() = default;
    std::tuple<double,double> univariateQuadraticEquation(const double a,const double b,const double c);
};


}
}

#endif //PREDICTION_COMMONMATH_H
