#ifndef ___CODER_G_FUN_API_H__
#define ___CODER_G_FUN_API_H__
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_G_fun_api.h"

extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;
extern void G_fun(real_T in1[7], real_T A[9]);
extern void G_fun_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void G_fun_atexit(void);
extern void G_fun_initialize(void);
extern void G_fun_terminate(void);
extern void G_fun_xil_terminate(void);

#endif
