#include "rt_nonfinite.h"
#include "p_ee_fun.h"

void p_ee_fun(const double in1[9], double A[3])
{
  double t2;
  double t3;
  double t4;
  double t6;
  double t7;
  double t9;
  double t10;
  double t11;
  double t12;
  double t17;
  double t19;
  double t20;
  double t26;
  t2 = cos(in1[5]);
  t3 = sin(in1[3]);
  t4 = sin(in1[5]);
  t6 = cos(in1[3]);
  t7 = sin(in1[4]);
  t9 = t3 * t4 + t2 * t6 * t7;
  t10 = cos(in1[6]);
  t11 = cos(in1[4]);
  t12 = sin(in1[6]);
  t17 = sin(in1[7]) * 0.24 + sin(in1[8]) * 0.35;
  t19 = t2 * t3 - t4 * t6 * t7;
  t20 = t10 * 0.05;
  t26 = (cos(in1[7]) * 0.24 + cos(in1[8]) * 0.35) - 0.05;
  A[0] = (((in1[0] + t26 * (t4 * t6 - t2 * t3 * t7)) + t17 * (t9 * t12 + t2 *
            t10 * t11)) - t9 * (t20 + 0.02)) + t2 * t11 * t12 * 0.05;
  A[1] = (((in1[1] - t26 * (t2 * t6 + t3 * t4 * t7)) - t17 * (t12 * t19 - t4 *
            t10 * t11)) + t19 * (t20 + 0.02)) + t4 * t11 * t12 * 0.05;
  A[2] = (((in1[2] - t17 * (t7 * t10 - t6 * t11 * t12)) - t7 * t12 * 0.05) - t6 *
          t11 * (t20 + 0.02)) - t3 * t11 * t26;
}

void p_ee_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void p_ee_fun_terminate()
{
}
