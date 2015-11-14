#include "rt_nonfinite.h"
#include "J_e_fun.h"

static double rdivide(double x, double y);
static double rdivide(double x, double y)
{
  return x / y;
}

void J_e_fun(const double in1[8], double A[54])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t8;
  double t9;
  double t10;
  double t11;
  double t13;
  double t15;
  double t16;
  double t18;
  double t19;
  double t21;
  double t23;
  double t25;
  double t28;
  double t32;
  double t33;
  double t34;
  double t35;
  double t38;
  double t41;
  double t44;
  double t46;
  double t55;
  double t48;
  double t50;
  double t52;
  double t53;
  double t54;
  double t57;
  double t61;
  double t64;
  double t65;
  double t68;
  double t71;
  double t74;
  double t77;
  double t80;
  double t84;
  double t86;
  double t88;
  double t90;
  double t96;
  double t100;
  double t101;
  double t102;
  double t104;
  double t106;
  double t109;
  double t111;
  double t114;
  double t116;
  double t120;
  double t123;
  double x[54];
  t2 = sin(in1[0]);
  t3 = sin(in1[2]);
  t4 = cos(in1[0]);
  t5 = cos(in1[2]);
  t6 = sin(in1[1]);
  t8 = t3 * t4 - t2 * t5 * t6;
  t9 = cos(in1[3]);
  t10 = sin(in1[3]);
  t11 = sin(in1[4]);
  t13 = sin(in1[6]);
  t15 = t11 * rdivide(7.0, 20.0) + t13 * rdivide(2.0, 5.0);
  t16 = cos(in1[1]);
  t18 = t9 * rdivide(1.0, 20.0) + rdivide(1.0, 10.0);
  t19 = cos(in1[4]);
  t21 = cos(in1[6]);
  t23 = (t19 * rdivide(7.0, 20.0) + t21 * rdivide(2.0, 5.0)) - rdivide(3.0, 50.0);
  t25 = t2 * t5 - t3 * t4 * t6;
  t28 = t2 * t3 + t4 * t5 * t6;
  t32 = t10 * t28 + t5 * t9 * t16;
  t33 = sin(in1[7]);
  t34 = cos(in1[7]);
  t35 = t8 * t13 * rdivide(2.0, 5.0);
  t38 = rdivide(1.0, t13 * t34 - t21 * t33);
  t41 = t4 * t5 + t2 * t3 * t6;
  t44 = t10 * t25 - t3 * t9 * t16;
  t46 = t11 * t34;
  t55 = t19 * t33;
  t48 = t46 - t55;
  t50 = cos(in1[5]);
  t52 = sin(in1[5]);
  t53 = t33 * t50 - t34 * t52;
  t54 = t13 * t41 * rdivide(2.0, 5.0) - t21 * t44 * rdivide(2.0, 5.0);
  t57 = t6 * t9 - t4 * t10 * t16;
  t61 = t21 * t57 * 0.4 - t2 * t13 * t16 * 0.4;
  t64 = sin(in1[6] + 0.52359877559829882);
  t65 = cos(in1[6] + 0.52359877559829882);
  t68 = t32 * t64 + t8 * t65;
  t71 = t44 * t64 + t41 * t65;
  t74 = t57 * t64 + t2 * t16 * t65;
  t77 = (t6 * t74 + t5 * t16 * t68) - t3 * t16 * t71;
  t80 = (t28 * t68 + t25 * t71) - t4 * t16 * t74;
  t84 = (t8 * t68 + t41 * t71) + t2 * t16 * t74;
  t86 = t8 * t64 - t32 * t65;
  t88 = t41 * t64 - t44 * t65;
  t90 = t57 * t65 - t2 * t16 * t64;
  t96 = (t6 * t90 + t3 * t16 * t88) - t5 * t16 * t86;
  t100 = (t28 * t86 + t25 * t88) + t4 * t16 * t90;
  t101 = t8 * t86;
  t102 = t41 * t88;
  t104 = t11 * t21 - t13 * t19;
  t106 = t13 * t50 - t21 * t52;
  t109 = t6 * t10 + t4 * t9 * t16;
  t111 = t9 * t28 - t5 * t10 * t16;
  t114 = t9 * t25 + t3 * t10 * t16;
  t116 = t5 * t16 * t111;
  t120 = (t28 * t111 + t25 * t114) + t4 * t16 * t109;
  t123 = (t8 * t111 + t41 * t114) - t2 * t16 * t109;
  x[0] = 1.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  x[4] = 0.0;
  x[5] = 0.0;
  x[6] = 0.0;
  x[7] = 1.0;
  x[8] = 0.0;
  x[9] = 0.0;
  x[10] = 0.0;
  x[11] = 0.0;
  x[12] = 0.0;
  x[13] = 0.0;
  x[14] = 1.0;
  x[15] = 0.0;
  x[16] = 0.0;
  x[17] = 0.0;
  x[18] = (-t8 * t18 - t23 * t28) + t8 * t10 * t15;
  x[19] = (t23 * t25 + t18 * t41) - t10 * t15 * t41;
  x[20] = (t2 * t16 * t18 - t4 * t16 * t23) - t2 * t10 * t15 * t16;
  x[21] = 0.0;
  x[22] = 0.0;
  x[23] = 0.0;
  x[24] = ((-t15 * (t5 * t6 * t9 - t4 * t5 * t10 * t16) - t5 * t6 * t10 * 0.05)
           - t4 * t5 * t16 * t18) - t2 * t5 * t16 * t23;
  x[25] = ((-t15 * (t3 * t6 * t9 - t3 * t4 * t10 * t16) - t3 * t6 * t10 * 0.05)
           - t3 * t4 * t16 * t18) - t2 * t3 * t16 * t23;
  x[26] = ((-t15 * (t9 * t16 + t4 * t6 * t10) - t10 * t16 * 0.05) + t4 * t6 *
           t18) + t2 * t6 * t23;
  x[27] = t10 * t38 * t104 * 0.875;
  x[28] = 0.0;
  x[29] = t9 * t38 * t104 * -0.875;
  x[30] = ((-t18 * t25 + t15 * t44) + t23 * t41) - t3 * t10 * t16 * 0.05;
  x[31] = ((t8 * t23 - t18 * t28) + t15 * t32) + t5 * t10 * t16 * 0.05;
  x[32] = 0.0;
  x[33] = t10 * t38 * t106 * 0.875;
  x[34] = 0.0;
  x[35] = t9 * t38 * t106 * -0.875;
  x[36] = (t10 * t28 * 0.05 + t15 * t111) + t5 * t9 * t16 * 0.05;
  x[37] = (t10 * t25 * -0.05 - t15 * t114) + t3 * t9 * t16 * 0.05;
  x[38] = (t6 * t9 * -0.05 + t15 * t109) + t4 * t10 * t16 * 0.05;
  x[39] = t77;
  x[40] = t96;
  x[41] = (t116 - t6 * t109) - t3 * t16 * t114;
  x[42] = (t8 * t11 * -0.35 + t19 * t32 * 0.35) + t38 * t48 * (t35 - t21 * t32 *
    0.4) * 0.875;
  x[43] = (t11 * t41 * 0.35 - t19 * t44 * 0.35) - t38 * t48 * t54 * 0.875;
  x[44] = (t19 * t57 * -0.35 + t2 * t11 * t16 * 0.35) + t38 * t61 * (t46 - t55) *
    0.875;
  x[45] = -t2 * t80 - t4 * t84;
  x[46] = t2 * t100 + t4 * ((t101 + t102) - t2 * t16 * t90);
  x[47] = -t2 * t120 - t4 * t123;
  x[48] = t38 * t53 * (t35 - t21 * t32 * 0.4) * 0.875;
  x[49] = t38 * t53 * t54 * -0.875;
  x[50] = t38 * t53 * t61 * 0.875;
  x[51] = (-t6 * t77 + t4 * t16 * t80) - t2 * t16 * t84;
  x[52] = (-t6 * t96 + t2 * t16 * ((t101 + t102) - t2 * t16 * t90)) - t4 * t16 *
    t100;
  x[53] = (t6 * ((-t116 + t6 * t109) + t3 * t16 * t114) + t4 * t16 * t120) - t2 *
    t16 * t123;
  memcpy(&A[0], &x[0], 54U * sizeof(double));
}

void J_e_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void J_e_fun_terminate()
{
}
