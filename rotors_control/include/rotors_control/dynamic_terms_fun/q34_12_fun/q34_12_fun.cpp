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
  double t10;
  double t14;
  double t28;
  t2 = cos(in1[0]);
  t3 = cos(in1[1]);
  t5 = tan(in1[0] * 0.5);
  t7 = tan(in1[1] * 0.5);
  t8 = t5 * t5;
  t9 = t7 * t7;
  t10 = t8 * t9 * 25.0;
  t14 = cos(in1[0] - in1[1]);
  t28 = (((t5 * -210.0 + t7 * 210.0) + t7 * t8 * 210.0) + 1.7320508075688772 *
         sqrt(((((t10 + t8 * 8.0) + t9 * 18.0) + t5 * t7 * 24.0) + 25.0) *
              ((((t10 + t8 * 841.0) + t9 * 361.0) - t5 * t7 * 1152.0) + 25.0)))
    - t5 * t9 * 210.0;
  A[0] = atan((t2 + 1.0) * (t3 + 1.0) * t28 * 0.25 / (((t2 * 135.0 - t3 * 135.0)
    + t14 * 72.0) - 122.0)) * -2.0;
  A[1] = atan((t2 + 1.0) * (t3 + 1.0) * t28 * -0.16666666666666666 / (((t2 *
    50.0 - t3 * 50.0) - t14 * 48.0) + 23.0)) * 2.0;
}

void q34_12_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void q34_12_fun_terminate()
{
}
