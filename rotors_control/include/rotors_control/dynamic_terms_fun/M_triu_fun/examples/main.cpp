#include "rt_nonfinite.h"
#include "M_triu_fun.h"
#include "main.h"

static void argInit_8x1_real_T(double result[8]);
static double argInit_real_T();
static void main_M_triu_fun();
static void argInit_8x1_real_T(double result[8])
{
  int idx0;
  for (idx0 = 0; idx0 < 8; idx0++) {
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_M_triu_fun()
{
  double dv0[8];
  double A[81];
  argInit_8x1_real_T(dv0);
  M_triu_fun(dv0, A);
}

int main(int, const char * const [])
{
  M_triu_fun_initialize();
  main_M_triu_fun();
  M_triu_fun_terminate();
  return 0;
}
