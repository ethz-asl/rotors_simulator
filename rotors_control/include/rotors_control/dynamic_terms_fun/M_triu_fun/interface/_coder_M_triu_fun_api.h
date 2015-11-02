#ifndef ___CODER_M_TRIU_FUN_API_H__
#define ___CODER_M_TRIU_FUN_API_H__
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_M_triu_fun_api.h"

extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;
extern void M_triu_fun(real_T in1[8], real_T A[81]);
extern void M_triu_fun_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void M_triu_fun_atexit(void);
extern void M_triu_fun_initialize(void);
extern void M_triu_fun_terminate(void);
extern void M_triu_fun_xil_terminate(void);

#endif
