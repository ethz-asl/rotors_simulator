#include "rt_nonfinite.h"
#include "q34_12_fun.h"
#include "main.h"

static void argInit_2x1_real_T(double result[2]);
static double argInit_real_T();
static void main_q34_12_fun();
static void argInit_2x1_real_T(double result[2])
{
  int idx0;
  for (idx0 = 0; idx0 < 2; idx0++) {
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_q34_12_fun()
{
  double dv0[2];
  double A[2];
  argInit_2x1_real_T(dv0);
  q34_12_fun(dv0, A);
}

int main(int, const char * const [])
{
  q34_12_fun_initialize();
  main_q34_12_fun();
  q34_12_fun_terminate();
  return 0;
}
