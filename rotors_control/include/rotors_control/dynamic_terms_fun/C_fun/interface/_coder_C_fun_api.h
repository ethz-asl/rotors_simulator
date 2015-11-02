#ifndef ___CODER_C_FUN_API_H__
#define ___CODER_C_FUN_API_H__
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_C_fun_api.h"

extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;
extern void C_fun(real_T in1[17], real_T A[81]);
extern void C_fun_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void C_fun_atexit(void);
extern void C_fun_initialize(void);
extern void C_fun_terminate(void);
extern void C_fun_xil_terminate(void);

#endif
