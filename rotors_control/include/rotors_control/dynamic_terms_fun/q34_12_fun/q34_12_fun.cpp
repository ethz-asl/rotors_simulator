#include "rt_nonfinite.h"
#include "q34_12_fun.h"

void q34_12_fun(const double in1[2], double A[2])
{
  double t2;
  double t3;
  double t5;
  double t7;
  double t8;
  double t9;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t20;
  double t23;
  t2 = cos(in1[0]);
  t3 = cos(in1[1]);
  t5 = tan(in1[0] * 0.5);
  t7 = tan(in1[1] * 0.5);
  t8 = t5 * t5;
  t9 = t7 * t7;
  t14 = cos(in1[0] - in1[1]) * 1225.0;
  t15 = t7 * 1400.0;
  t16 = t7 * t8 * 1400.0;
  t17 = t8 * 1681.0;
  t18 = t9 * 841.0;
  t19 = t8 * t9 * 36.0;
  t20 = t5 * t7 * 2450.0;
  t23 = (((t8 * -81.0 + t20) + t9 * 759.0) + t8 * t9 * 1564.0) + 1564.0;
  A[0] = atan((t2 + 1.0) * (t3 + 1.0) * ((((t5 * -1400.0 + t15) + t16) + sqrt
    (t23 * ((((t17 + t18) + t19) - t5 * t7 * 2450.0) + 36.0))) - t5 * t9 *
    1400.0) * 0.5 / (((t2 * 1820.0 - t3 * 1820.0) + t14) - 1777.0)) * -2.0;
  A[1] = atan((t2 + 1.0) * (t3 + 1.0) * ((((t5 * -1400.0 + t15) + t16) - t5 * t9
    * 1400.0) + sqrt(t23 * ((((t17 + t18) + t19) - t20) + 36.0))) * -0.5 / (((t2
    * 980.0 - t3 * 980.0) - t14) + 817.0)) * 2.0;
}

void q34_12_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void q34_12_fun_terminate()
{
}
