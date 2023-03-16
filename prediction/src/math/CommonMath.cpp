//
// Created by ai on 23-3-16.
//

#include "CommonMath.h"

namespace aptiv{
namespace math{
    using namespace std;
std::tuple<double, double> CommonMath::univariateQuadraticEquation(const double a_, const double b_, const double c_) {
    double discriminant, realPart, imaginaryPart;
    double a,b,c,x1,x2;
    a = a_;
    b = b_;
    c = c_;
    discriminant = b*b - 4*a*c;

    if (discriminant > 0) {
        x1 = (-b + sqrt(discriminant)) / (2*a);
        x2 = (-b - sqrt(discriminant)) / (2*a);
//        cout << "Roots are real and different." << endl;
//        cout << "x1 = " << x1 << endl;
//        cout << "x2 = " << x2 << endl;
    }

    else if (discriminant == 0) {
//        cout << "实根相同：" << endl;
        x1 = (-b + sqrt(discriminant)) / (2*a);
//        cout << "x1 = x2 =" << x1 << endl;
    }

    else {
        realPart = -b/(2*a);
        imaginaryPart =sqrt(-discriminant)/(2*a);
//        cout << "实根不同："  << endl;
//        cout << "x1 = " << realPart << "+" << imaginaryPart << "i" << endl;
//        cout << "x2 = " << realPart << "-" << imaginaryPart << "i" << endl;
    }
    return {x1,x2};
}

}
}