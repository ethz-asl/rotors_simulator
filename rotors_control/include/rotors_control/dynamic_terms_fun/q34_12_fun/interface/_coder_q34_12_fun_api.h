#ifndef ___CODER_Q34_12_FUN_API_H__
#define ___CODER_Q34_12_FUN_API_H__
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_q34_12_fun_api.h"

extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;
extern void q34_12_fun(real_T in1[2], real_T A[2]);
extern void q34_12_fun_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void q34_12_fun_atexit(void);
extern void q34_12_fun_initialize(void);
extern void q34_12_fun_terminate(void);
extern void q34_12_fun_xil_terminate(void);

#endif
