#include "rt_nonfinite.h"
#include "C_fun.h"
#include "main.h"

static void argInit_17x1_real_T(double result[17]);
static double argInit_real_T();
static void main_C_fun();
static void argInit_17x1_real_T(double result[17])
{
  int idx0;
  for (idx0 = 0; idx0 < 17; idx0++) {
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_C_fun()
{
  double dv0[17];
  double A[81];
  argInit_17x1_real_T(dv0);
  C_fun(dv0, A);
}

int main(int, const char * const [])
{
  C_fun_initialize();
  main_C_fun();
  C_fun_terminate();
  return 0;
}
