#ifndef ___CODER_P_EE_FUN_API_H__
#define ___CODER_P_EE_FUN_API_H__
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_p_ee_fun_api.h"

extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;
extern void p_ee_fun(real_T in1[9], real_T A[3]);
extern void p_ee_fun_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void p_ee_fun_atexit(void);
extern void p_ee_fun_initialize(void);
extern void p_ee_fun_terminate(void);
extern void p_ee_fun_xil_terminate(void);

#endif
