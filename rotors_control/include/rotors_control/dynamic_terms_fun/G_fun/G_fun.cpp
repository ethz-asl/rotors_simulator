#include "rt_nonfinite.h"
#include "G_fun.h"

void G_fun(const double in1[7], double A[9])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t10;
  double t12;
  double t14;
  double t16;
  double t18;
  double t20;
  double t22;
  double t23;
  double t25;
  double t27;
  double t30;
  double t32;
  double t34;
  double t36;
  double t39;
  double t41;
  double t44;
  double t48;
  double t50;
  t2 = cos(in1[0]);
  t3 = cos(in1[1]);
  t4 = cos(in1[3]);
  t5 = cos(in1[4]);
  t6 = sin(in1[0]);
  t7 = sin(in1[2]);
  t8 = sin(in1[3]);
  t9 = sin(in1[4]);
  t10 = cos(in1[2]);
  t12 = sin(in1[1]);
  t14 = t3 * t10 + t2 * t7 * t12;
  t16 = sin(in1[5]);
  t18 = t8 * 0.35 + t16 * 0.2;
  t20 = sin(in1[6]);
  t22 = t9 * 0.35 + t20 * 0.2;
  t23 = t10 * 0.05;
  t25 = t4 * 0.175;
  t27 = t5 * 0.175;
  t30 = cos(in1[5]);
  t32 = (t4 * 0.35 + t30 * 0.2) - 0.06;
  t34 = cos(in1[6]);
  t36 = (t5 * 0.35 + t34 * 0.2) + 0.06;
  t39 = t7 * t12 + t2 * t3 * t10;
  t41 = t10 * t12 - t2 * t3 * t7;
  t44 = 1.0 / (t16 * t34 - t20 * t30);
  t48 = t34 * t41 * 0.2 - t3 * t6 * t20 * 0.2;
  t50 = t30 * t41 * 0.2 - t3 * t6 * t16 * 0.2;
  A[0] = 0.0;
  A[1] = 0.0;
  A[2] = 29.37114;
  A[3] = (((((((t2 * t3 * (t25 - 0.06) * -0.11772 - t2 * t3 * (t27 + 0.06) *
                0.11772) + t3 * t6 * (t23 + 0.1) * 1.90314) - t2 * t3 * t32 *
              0.14715) - t2 * t3 * t36 * 0.14715) - t3 * t6 * t7 * t8 * 0.020601)
           - t3 * t6 * t7 * t9 * 0.020601) - t3 * t6 * t7 * t18 * 0.14715) - t3 *
    t6 * t7 * t22 * 0.14715;
  A[4] = ((((((((t3 * t7 * -0.095157 - t8 * t14 * 0.020601) - t9 * t14 *
                0.020601) - t14 * t18 * 0.14715) - t14 * t22 * 0.14715) + t2 *
             t12 * (t23 + 0.1) * 1.90314) + t6 * t12 * (t25 - 0.06) * 0.11772) +
           t6 * t12 * (t27 + 0.06) * 0.11772) + t6 * t12 * t32 * 0.14715) + t6 *
    t12 * t36 * 0.14715;
  A[5] = 0.0;
  A[6] = ((((t10 * t12 * -0.095157 + t8 * t39 * 0.020601) + t9 * t39 * 0.020601)
           + t18 * t39 * 0.14715) + t22 * t39 * 0.14715) + t2 * t3 * t7 *
    0.095157;
  A[7] = ((t4 * t41 * -0.0721035 - t44 * t48 * (t4 * t16 - t8 * t30) *
           0.12875625) - t44 * t50 * (t4 * t20 - t8 * t34) * 0.12875625) + t3 *
    t6 * t8 * 0.0721035;
  A[8] = ((t5 * t41 * -0.0721035 + t44 * t48 * (t5 * t16 - t9 * t30) *
           0.12875625) + t44 * t50 * (t5 * t20 - t9 * t34) * 0.12875625) + t3 *
    t6 * t9 * 0.0721035;
}

void G_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void G_fun_terminate()
{
}
