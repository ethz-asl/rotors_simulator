#ifndef ___CODER_C_FUN_MEX_H__
#define ___CODER_C_FUN_MEX_H__
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "_coder_C_fun_api.h"

extern void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const
  mxArray *prhs[]);
extern emlrtCTX mexFunctionCreateRootTLS(void);

#endif
