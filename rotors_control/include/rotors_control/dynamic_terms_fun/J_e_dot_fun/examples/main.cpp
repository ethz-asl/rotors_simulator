#include "rt_nonfinite.h"
#include "J_e_dot_fun.h"
#include "main.h"

static void argInit_14x1_real_T(double result[14]);
static void argInit_9x1_real_T(double result[9]);
static double argInit_real_T();
static void main_J_e_dot_fun();
static void main_p_ee_fun();
static void argInit_14x1_real_T(double result[14])
{
  int idx0;
  for (idx0 = 0; idx0 < 14; idx0++) {
    result[idx0] = argInit_real_T();
  }
}

static void argInit_9x1_real_T(double result[9])
{
  int idx0;
  for (idx0 = 0; idx0 < 9; idx0++) {
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_J_e_dot_fun()
{
  double dv0[14];
  double A[54];
  argInit_14x1_real_T(dv0);
  J_e_dot_fun(dv0, A);
}

static void main_p_ee_fun()
{
  double dv1[9];
  double A[3];
  argInit_9x1_real_T(dv1);
  p_ee_fun(dv1, A);
}

int main(int, const char * const [])
{
  J_e_dot_fun_initialize();
  main_J_e_dot_fun();
  main_p_ee_fun();
  J_e_dot_fun_terminate();
  return 0;
}
