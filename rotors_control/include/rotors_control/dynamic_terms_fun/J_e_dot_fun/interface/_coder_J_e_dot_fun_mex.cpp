#include "_coder_J_e_dot_fun_api.h"
#include "_coder_J_e_dot_fun_mex.h"

static const char * emlrtEntryPoints[2] = { "J_e_dot_fun", "p_ee_fun" };

static void J_e_dot_fun_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[1]);
static void p_ee_fun_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[1]);
static void J_e_dot_fun_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[1])
{
  int32_T n;
  const mxArray *inputs[1];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        11, "J_e_dot_fun");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 11,
                        "J_e_dot_fun");
  }

  for (n = 0; n < nrhs; n++) {
    inputs[n] = prhs[n];
  }

  J_e_dot_fun_api(inputs, outputs);
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
  J_e_dot_fun_terminate();
}

static void p_ee_fun_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[1])
{
  int32_T n;
  const mxArray *inputs[1];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4, 8,
                        "p_ee_fun");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 8,
                        "p_ee_fun");
  }

  for (n = 0; n < nrhs; n++) {
    inputs[n] = prhs[n];
  }

  p_ee_fun_api(inputs, outputs);
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);
  J_e_dot_fun_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(J_e_dot_fun_atexit);
  J_e_dot_fun_initialize();
  switch (emlrtGetEntryPointIndex(nrhs, prhs, emlrtEntryPoints, 2)) {
   case 0:
    J_e_dot_fun_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 1:
    p_ee_fun_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}
