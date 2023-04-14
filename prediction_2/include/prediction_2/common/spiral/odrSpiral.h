//
// Created by ai on 23-4-14.
//

#ifndef PREDICTION_2_INCLUDE_PREDICTION_2_COMMON_SPIRAL_ODRSPIRAL_H_
#define PREDICTION_2_INCLUDE_PREDICTION_2_COMMON_SPIRAL_ODRSPIRAL_H_

/**
* compute the actual "standard" spiral, starting with curvature 0
* @param s      run-length along spiral
* @param cDot   first derivative of curvature [1/m2]
* @param x      resulting x-coordinate in spirals local co-ordinate system [m]
* @param y      resulting y-coordinate in spirals local co-ordinate system [m]
* @param t      tangent direction at s [rad]
*/

extern void odrSpiral( double s, double cDot, double *x, double *y, double *t );
#endif //PREDICTION_2_INCLUDE_PREDICTION_2_COMMON_SPIRAL_ODRSPIRAL_H_
