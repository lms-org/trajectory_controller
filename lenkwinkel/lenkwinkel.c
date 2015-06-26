/*
 * File: lenkwinkel.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Jun-2015 19:58:43
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "lenkwinkel.h"
#include <stdio.h>
/* Function Definitions */

/*
 * Arguments    : double x_s
 *                double y_s
 *                double phi_s
 *                unsigned char chooser
 *                double *delta_h
 *                double *delta_v
 * Return Type  : void
 */
void lenkwinkel(double x_s, double y_s, double phi_s, unsigned char chooser,
                double *delta_h, double *delta_v)
{
  double s;
  s = sqrt(x_s * x_s + y_s * y_s);
  *delta_h = 0.0;
  *delta_v = 0.0;
  printf("chooser: %d \n",chooser);
  if (chooser == 1) {
    *delta_h = -(0.01333 * (((((((6.603E+8 * phi_s * (s * s) * exp(6.189 * s) -
      2.211E+9 * (s * s) * y_s * exp(6.189 * s)) - 6.603E+8 * phi_s * (s * s) *
      exp(0.4464 * s)) - 2.308E+8 * (s * s) * y_s * exp(0.4464 * s)) - 6.603E+8 *
      phi_s * (s * s) * exp(3.977 * s)) + 2.308E+8 * (s * s) * y_s * exp(3.977 *
      s)) + 6.603E+8 * phi_s * (s * s) * exp(-1.766 * s)) + 2.211E+9 * (s * s) *
      y_s * exp(-1.766 * s))) / (s * s * ((((2.279E+7 * exp(2.212 * s) -
      3.329E+7 * exp(4.424 * s)) + 2.189E+7 * exp(7.955 * s)) + 2.189E+7 * exp
      (-3.531 * s)) - 3.329E+7));
    *delta_v = atan(0.002 * ((((((((((((1.693E+10 * y_s * exp(6.189 * s) -
      5.056E+9 * phi_s * exp(6.189 * s)) - 1.664E+10 * sin(*delta_h)) -
      4.845E+10 * phi_s * exp(0.4464 * s)) - 1.693E+10 * y_s * exp(0.4464 * s))
      + 4.845E+10 * phi_s * exp(3.977 * s)) - 1.693E+10 * y_s * exp(3.977 * s))
      + 5.056E+9 * phi_s * exp(-1.766 * s)) + 1.693E+10 * y_s * exp(-1.766 * s))
      + 1.139E+10 * exp(2.212 * s) * sin(*delta_h)) - 1.664E+10 * exp(4.424 * s)
      * sin(*delta_h)) + 1.095E+10 * exp(7.955 * s) * sin(*delta_h)) + 1.095E+10
      * exp(-3.531 * s) * sin(*delta_h)) / (cos(*delta_h) * ((((2.279E+7 * exp
      (2.212 * s) - 3.329E+7 * exp(4.424 * s)) + 2.189E+7 * exp(7.955 * s)) +
      2.189E+7 * exp(-3.531 * s)) - 3.329E+7)));
  } else if (chooser == 2) {
    *delta_h = 0.06667 * (((((((1.828E+9 * phi_s * (s * s) * exp(4.041 * s) -
      2.443E+8 * (s * s) * y_s * exp(4.041 * s)) + 1.828E+9 * phi_s * (s * s) *
      exp(2.487 * s)) + 2.443E+8 * (s * s) * y_s * exp(2.487 * s)) - 1.828E+9 *
      phi_s * (s * s) * exp(7.306 * s)) + 7.163E+9 * (s * s) * y_s * exp(7.306 *
      s)) - 1.828E+9 * phi_s * (s * s) * exp(-0.7771 * s)) - 7.163E+9 * (s * s) *
                          y_s * exp(-0.7771 * s)) / (s * s * ((((8.712E+8 * exp
      (8.083 * s) + 2.547E+8 * exp(3.264 * s)) - 9.985E+8 * exp(6.529 * s)) +
      8.712E+8 * exp(-1.554 * s)) - 9.985E+8));
    *delta_v = atan(0.002 * ((((((((((((4.356E+11 * exp(-1.554 * s) * sin
      (*delta_h) - 4.993E+11 * sin(*delta_h)) + 1.755E+12 * phi_s * exp(4.041 *
      s)) - 2.344E+11 * y_s * exp(4.041 * s)) - 1.755E+12 * phi_s * exp(2.487 *
      s)) - 2.344E+11 * y_s * exp(2.487 * s)) - 5.983E+10 * phi_s * exp(7.306 *
      s)) + 2.344E+11 * y_s * exp(7.306 * s)) + 5.983E+10 * phi_s * exp(-0.7771 *
      s)) + 2.344E+11 * y_s * exp(-0.7771 * s)) + 4.356E+11 * exp(8.083 * s) *
      sin(*delta_h)) + 1.274E+11 * exp(3.264 * s) * sin(*delta_h)) - 4.993E+11 *
      exp(6.529 * s) * sin(*delta_h)) / (cos(*delta_h) * ((((8.712E+8 * exp
      (8.083 * s) + 2.547E+8 * exp(3.264 * s)) - 9.985E+8 * exp(6.529 * s)) +
      8.712E+8 * exp(-1.554 * s)) - 9.985E+8)));
  } else {
    if (chooser == 3) {
      *delta_h = 0.06667 * (((((((1.512E+10 * phi_s * (s * s) * exp(4.051 * s) -
        9.843E+8 * (s * s) * y_s * exp(4.051 * s)) + 1.512E+10 * phi_s * (s * s)
        * exp(3.276 * s)) + 9.843E+8 * (s * s) * y_s * exp(3.276 * s)) -
        1.512E+10 * phi_s * (s * s) * exp(7.715 * s)) + 6.079E+10 * (s * s) *
        y_s * exp(7.715 * s)) - 1.512E+10 * phi_s * (s * s) * exp(-0.3876 * s))
                            - 6.079E+10 * (s * s) * y_s * exp(-0.3876 * s)) / (s
        * s * ((((1.523E+10 * exp(8.102 * s) + 2.038E+9 * exp(3.663 * s)) -
                 1.625E+10 * exp(7.327 * s)) + 1.523E+10 * exp(-0.7752 * s)) -
               1.625E+10));
      *delta_v = atan(0.001 * ((((((((((((1.523E+13 * exp(-0.7752 * s) * sin
        (*delta_h) - 1.625E+13 * sin(*delta_h)) + 5.956E+13 * phi_s * exp(4.051 *
        s)) - 3.877E+12 * y_s * exp(4.051 * s)) - 5.956E+13 * phi_s * exp(3.276 *
        s)) - 3.877E+12 * y_s * exp(3.276 * s)) - 9.645E+11 * phi_s * exp(7.715 *
        s)) + 3.877E+12 * y_s * exp(7.715 * s)) + 9.645E+11 * phi_s * exp
        (-0.3876 * s)) + 3.877E+12 * y_s * exp(-0.3876 * s)) + 1.523E+13 * exp
        (8.102 * s) * sin(*delta_h)) + 2.038E+12 * exp(3.663 * s) * sin(*delta_h))
        - 1.625E+13 * exp(7.327 * s) * sin(*delta_h)) / (cos(*delta_h) *
        ((((1.523E+10 * exp(8.102 * s) + 2.038E+9 * exp(3.663 * s)) - 1.625E+10 *
           exp(7.327 * s)) + 1.523E+10 * exp(-0.7752 * s)) - 1.625E+10)));
    }
  }
}

/*
 * File trailer for lenkwinkel.c
 *
 * [EOF]
 */
