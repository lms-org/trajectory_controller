/*
 * File: _coder_lenkwinkel_api.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 26-Jun-2015 19:58:43
 */

/* Include Files */
#include "_coder_lenkwinkel_api.h"

/* Function Declarations */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *x_s, const
  char_T *identifier);
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static uint8_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *chooser,
  const char_T *identifier);
static uint8_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const real_T u);
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static uint8_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void lenkwinkel_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void lenkwinkel_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void lenkwinkel_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  lenkwinkel_xil_terminate();
}

/*
 * Arguments    : const mxArray * const prhs[4]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void lenkwinkel_api(const mxArray * const prhs[4], const mxArray *plhs[2])
{
  real_T x_s;
  real_T y_s;
  real_T phi_s;
  uint8_T chooser;
  real_T delta_v;
  real_T delta_h;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  x_s = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "x_s");
  y_s = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "y_s");
  phi_s = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "phi_s");
  chooser = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "chooser");

  /* Invoke the target function */
  lenkwinkel(x_s, y_s, phi_s, chooser, &delta_h, &delta_v);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(delta_h);
  plhs[1] = emlrt_marshallOut(delta_v);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *x_s
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *x_s, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(x_s), &thisId);
  emlrtDestroyArray(&x_s);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *chooser
 *                const char_T *identifier
 * Return Type  : uint8_T
 */
static uint8_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *chooser,
  const char_T *identifier)
{
  uint8_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(chooser), &thisId);
  emlrtDestroyArray(&chooser);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint8_T
 */
static uint8_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  uint8_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint8_T
 */
static uint8_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  uint8_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 0U, 0);
  ret = *(uint8_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_lenkwinkel_api.c
 *
 * [EOF]
 */
