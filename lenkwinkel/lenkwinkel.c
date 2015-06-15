/*
 * File: lenkwinkel.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 15-Jun-2015 17:03:28
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "lenkwinkel.h"

/* Function Definitions */

/*
 * Arguments    : double x_s
 *                double y_s
 *                double phi_s
 *                double *delta_h
 *                double *delta_v
 * Return Type  : void
 */
void lenkwinkel(double x_s, double y_s, double phi_s, double *delta_h, double
                *delta_v)
{
  double s;
  s = sqrt(x_s * x_s + y_s * y_s);
  *delta_h = -(0.01333 * (((((((6.603E+8 * phi_s * (s * s) * exp(6.189 * s) -
    2.211E+9 * (s * s) * y_s * exp(6.189 * s)) - 6.603E+8 * phi_s * (s * s) *
    exp(0.4464 * s)) - 2.308E+8 * (s * s) * y_s * exp(0.4464 * s)) - 6.603E+8 *
    phi_s * (s * s) * exp(3.977 * s)) + 2.308E+8 * (s * s) * y_s * exp(3.977 * s))
    + 6.603E+8 * phi_s * (s * s) * exp(-1.766 * s)) + 2.211E+9 * (s * s) * y_s *
    exp(-1.766 * s))) / (s * s * ((((2.279E+7 * exp(2.212 * s) - 3.329E+7 * exp
    (4.424 * s)) + 2.189E+7 * exp(7.955 * s)) + 2.189E+7 * exp(-3.531 * s)) -
    3.329E+7));
  *delta_v = atan(0.002 * ((((((((((((1.693E+10 * y_s * exp(6.189 * s) -
    5.056E+9 * phi_s * exp(6.189 * s)) - 1.664E+10 * sin(*delta_h)) - 4.845E+10 *
    phi_s * exp(0.4464 * s)) - 1.693E+10 * y_s * exp(0.4464 * s)) + 4.845E+10 *
    phi_s * exp(3.977 * s)) - 1.693E+10 * y_s * exp(3.977 * s)) + 5.056E+9 *
    phi_s * exp(-1.766 * s)) + 1.693E+10 * y_s * exp(-1.766 * s)) + 1.139E+10 *
    exp(2.212 * s) * sin(*delta_h)) - 1.664E+10 * exp(4.424 * s) * sin(*delta_h))
    + 1.095E+10 * exp(7.955 * s) * sin(*delta_h)) + 1.095E+10 * exp(-3.531 * s) *
    sin(*delta_h)) / (cos(*delta_h) * ((((2.279E+7 * exp(2.212 * s) - 3.329E+7 *
    exp(4.424 * s)) + 2.189E+7 * exp(7.955 * s)) + 2.189E+7 * exp(-3.531 * s)) -
    3.329E+7)));
}

/*
 * File trailer for lenkwinkel.c
 *
 * [EOF]
 */
