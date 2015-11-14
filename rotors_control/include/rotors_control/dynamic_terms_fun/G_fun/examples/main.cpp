#include "rt_nonfinite.h"
#include "G_fun.h"
#include "main.h"

static void argInit_7x1_real_T(double result[7]);
static double argInit_real_T();
static void main_G_fun();
static void argInit_7x1_real_T(double result[7])
{
  int idx0;
  for (idx0 = 0; idx0 < 7; idx0++) {
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_G_fun()
{
  double dv0[7];
  double A[9];
  argInit_7x1_real_T(dv0);
  G_fun(dv0, A);
}

int main(int, const char * const [])
{
  G_fun_initialize();
  main_G_fun();
  G_fun_terminate();
  return 0;
}
