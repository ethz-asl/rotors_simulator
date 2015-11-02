#include "rt_nonfinite.h"
#include "J_e_fun.h"
#include "main.h"

static void argInit_8x1_real_T(double result[8]);
static double argInit_real_T();
static void main_J_e_fun();
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

static void main_J_e_fun()
{
  double dv0[8];
  double A[54];
  argInit_8x1_real_T(dv0);
  J_e_fun(dv0, A);
}

int main(int, const char * const [])
{
  J_e_fun_initialize();
  main_J_e_fun();
  J_e_fun_terminate();
  return 0;
}
