/* 
 * File: _coder_lenkwinkel_api.h 
 *  
 * MATLAB Coder version            : 2.7 
 * C/C++ source code generated on  : 15-Jun-2015 17:03:28 
 */

#ifndef ___CODER_LENKWINKEL_API_H__
#define ___CODER_LENKWINKEL_API_H__
/* Include Files */ 
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */ 
extern void lenkwinkel_initialize(emlrtContext *aContext);
extern void lenkwinkel_terminate(void);
extern void lenkwinkel_atexit(void);
extern void lenkwinkel_api(const mxArray * const prhs[3], const mxArray *plhs[2]);
extern void lenkwinkel(real_T x_s, real_T y_s, real_T phi_s, real_T *delta_h, real_T *delta_v);
extern void lenkwinkel_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_lenkwinkel_api.h 
 *  
 * [EOF] 
 */
