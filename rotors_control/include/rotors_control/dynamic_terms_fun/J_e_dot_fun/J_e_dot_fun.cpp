#include "rt_nonfinite.h"
#include "J_e_dot_fun.h"

static double rdivide(double x, double y);
static double rt_powd_snf(double u0, double u1);
static double rdivide(double x, double y)
{
  return x / y;
}

static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

void J_e_dot_fun(const double in1[14], double A[54])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t10;
  double t11;
  double t12;
  double t14;
  double t16;
  double t18;
  double t20;
  double t22;
  double t24;
  double t27;
  double t30;
  double t31;
  double t32;
  double t33;
  double t45;
  double t34;
  double t35;
  double t38;
  double t41;
  double t43;
  double t44;
  double t48;
  double t49;
  double t51;
  double t52;
  double t54;
  double t59;
  double t61;
  double t72;
  double t64;
  double t78;
  double t65;
  double t66;
  double t67;
  double t68;
  double t71;
  double t73;
  double t76;
  double t77;
  double t80;
  double t83;
  double t86;
  double t89;
  double t92;
  double t95;
  double t98;
  double t99;
  double t135;
  double t100;
  double t101;
  double t102;
  double t104;
  double t103;
  double t107;
  double t108;
  double t111;
  double t112;
  double t117;
  double t113;
  double t116;
  double t118;
  double t120;
  double t123;
  double t124;
  double a;
  double t125;
  double t126;
  double t179;
  double t127;
  double t130;
  double t132;
  double t133;
  double t134;
  double t138;
  double t142;
  double t144;
  double t145;
  double t146;
  double t151;
  double t154;
  double t158;
  double t159;
  double t162;
  double t165;
  double t168;
  double t172;
  double t174;
  double t175;
  double t176;
  double t178;
  double t185;
  double t180;
  double t183;
  double t184;
  double t188;
  double t189;
  double t190;
  double t191;
  double t192;
  double t194;
  double t195;
  double t196;
  double t199;
  double t202;
  double t205;
  double t208;
  double t209;
  double t211;
  double t212;
  double t215;
  double t218;
  double t220;
  double t223;
  double t225;
  double t226;
  double t227;
  double t230;
  double t231;
  double t232;
  double t233;
  double t235;
  double t236;
  double t237;
  double t238;
  double t240;
  double t243;
  double t244;
  double t246;
  double t247;
  double t248;
  double t251;
  double t252;
  double t253;
  double t255;
  double t256;
  double t257;
  double t294;
  double t258;
  double t259;
  double t295;
  double t296;
  double t339;
  double t261;
  double t264;
  double t267;
  double t269;
  double t271;
  double t274;
  double t277;
  double t280;
  double t281;
  double t282;
  double t283;
  double t284;
  double t287;
  double t290;
  double t292;
  double t298;
  double t299;
  double t340;
  double t300;
  double t302;
  double t303;
  double t304;
  double t305;
  double t307;
  double t310;
  double t311;
  double t316;
  double t312;
  double t313;
  double t314;
  double t315;
  double t320;
  double t324;
  double t328;
  double t331;
  double t335;
  double t338;
  double t343;
  double t347;
  double t350;
  double t352;
  double t355;
  double t357;
  double t360;
  double t365;
  double t368;
  double t372;
  double t375;
  double t378;
  double t382;
  double t385;
  double t389;
  double t391;
  double t395;
  double t399;
  double t402;
  double t403;
  double t404;
  double t405;
  double t406;
  double t407;
  double t408;
  double t410;
  double t411;
  double t412;
  double t413;
  double t414;
  double t415;
  double t419;
  double t420;
  double t421;
  double t422;
  double t426;
  double t427;
  double t446;
  double t428;
  double t431;
  double t434;
  double t438;
  double t441;
  double t445;
  double t447;
  double t448;
  double x[54];
  t2 = cos(in1[2]);
  t3 = cos(in1[1]);
  t4 = sin(in1[0]);
  t5 = sin(in1[3]);
  t6 = cos(in1[0]);
  t7 = cos(in1[3]);
  t8 = sin(in1[2]);
  t10 = sin(in1[1]);
  t11 = t6 * t8 - t2 * t4 * t10;
  t12 = sin(in1[4]);
  t14 = sin(in1[6]);
  t16 = t12 * rdivide(6.0, 25.0) + t14 * rdivide(7.0, 20.0);
  t18 = t7 * rdivide(1.0, 20.0) + rdivide(1.0, 50.0);
  t20 = cos(in1[4]);
  t22 = cos(in1[6]);
  t24 = (t20 * rdivide(6.0, 25.0) + t22 * rdivide(7.0, 20.0)) - rdivide(1.0,
    20.0);
  t27 = t4 * t8 + t2 * t6 * t10;
  t30 = t2 * t6 + t4 * t8 * t10;
  t31 = sin(in1[7]);
  t32 = cos(in1[7]);
  t33 = t14 * t32;
  t45 = t22 * t31;
  t34 = t33 - t45;
  t35 = rdivide(1.0, t34);
  t38 = t14 * t27 * rdivide(7.0, 20.0) + t5 * t11 * t22 * rdivide(7.0, 20.0);
  t41 = (t2 * t3 * t6 * t24 + t2 * t3 * t4 * t5 * t16) - t2 * t3 * t4 * t18;
  t43 = t2 * t7 * t10 - t2 * t3 * t5 * t6;
  t44 = t12 * t32;
  t48 = t22 * t43 * rdivide(7.0, 20.0) - t2 * t3 * t4 * t14 * rdivide(7.0, 20.0);
  t49 = cos(in1[5]);
  t51 = sin(in1[5]);
  t52 = t31 * t49 - t32 * t51;
  t54 = t7 * t8 * t10 - t3 * t5 * t6 * t8;
  t59 = ((t16 * t54 + t5 * t8 * t10 * rdivide(1.0, 20.0)) + t3 * t6 * t8 * t18)
    + t3 * t4 * t8 * t24;
  t61 = t2 * t4 - t6 * t8 * t10;
  t72 = t3 * t7 * t8;
  t64 = t5 * t61 - t72;
  t78 = t20 * t31;
  t65 = t44 - t78;
  t66 = t2 * t3 * t7;
  t67 = t11 * t24;
  t68 = t2 * t3 * t5 * rdivide(1.0, 20.0);
  t71 = (t18 * t30 + t24 * t61) - t5 * t16 * t30;
  t73 = t14 * t30 * rdivide(7.0, 20.0);
  t76 = t66 + t5 * t27;
  t77 = t16 * t76;
  t80 = t7 * t27 - t2 * t3 * t5;
  t83 = t7 * t61 + t3 * t5 * t8;
  t86 = (t16 * t83 + t5 * t61 * rdivide(1.0, 20.0)) - t3 * t7 * t8 * rdivide(1.0,
    20.0);
  t89 = t2 * t5 * t10 + t2 * t3 * t6 * t7;
  t92 = (t16 * t89 + t2 * t3 * t5 * t6 * rdivide(1.0, 20.0)) - t2 * t7 * t10 *
    rdivide(1.0, 20.0);
  t95 = t5 * t11 * rdivide(1.0, 20.0) + t7 * t11 * t16;
  t98 = t20 * t80 * rdivide(6.0, 25.0) - t22 * t35 * t65 * t80 * rdivide(6.0,
    25.0);
  t99 = t20 * t64 * rdivide(6.0, 25.0);
  t135 = t22 * t64 * rdivide(7.0, 20.0);
  t100 = t73 - t135;
  t101 = t35 * t65 * t100 * rdivide(24.0, 35.0);
  t102 = t11 * t14 * rdivide(7.0, 20.0);
  t104 = t22 * t76 * rdivide(7.0, 20.0);
  t103 = t102 - t104;
  t107 = t22 * t32 + t14 * t31;
  t108 = rdivide(1.0, t34 * t34);
  t111 = t20 * t32 + t12 * t31;
  t112 = t35 * t103 * t111 * rdivide(24.0, 35.0);
  t117 = t65 * t103 * t107 * t108 * rdivide(24.0, 35.0);
  t113 = t112 - t117;
  t116 = t14 * t76 * rdivide(7.0, 20.0) + t11 * t22 * rdivide(7.0, 20.0);
  t118 = t35 * t65 * t116 * rdivide(24.0, 35.0);
  t120 = t2 * t3 * t4 * t12 * rdivide(6.0, 25.0);
  t123 = (t12 * t27 * rdivide(6.0, 25.0) + t5 * t11 * t20 * rdivide(6.0, 25.0))
    - t35 * t38 * t65 * rdivide(24.0, 35.0);
  t124 = t12 * t22;
  a = t33 - t45;
  t125 = rdivide(1.0, a * a);
  t126 = t52 * t107 * t125 * (t102 - t104) * rdivide(24.0, 35.0);
  t179 = t14 * t20;
  t127 = t124 - t179;
  t130 = t32 * t49 + t31 * t51;
  t132 = t14 * t49 - t22 * t51;
  t133 = t126 - t35 * t52 * t116 * rdivide(24.0, 35.0);
  t134 = t35 * t130 * (t102 - t104) * rdivide(24.0, 35.0);
  t138 = t14 * t61 * rdivide(7.0, 20.0) + t5 * t22 * t30 * rdivide(7.0, 20.0);
  t142 = (t3 * t6 * t8 * t24 + t3 * t4 * t5 * t8 * t16) - t3 * t4 * t8 * t18;
  t144 = t22 * t54 * rdivide(7.0, 20.0) - t3 * t4 * t8 * t14 * rdivide(7.0, 20.0);
  t145 = t24 * t30;
  t146 = t3 * t5 * t8 * rdivide(1.0, 20.0);
  t151 = ((t16 * t43 + t2 * t5 * t10 * rdivide(1.0, 20.0)) + t2 * t3 * t6 * t18)
    + t2 * t3 * t4 * t24;
  t154 = (t11 * t18 + t24 * t27) - t5 * t11 * t16;
  t158 = (t16 * t80 + t5 * t27 * rdivide(1.0, 20.0)) + t2 * t3 * t7 * rdivide
    (1.0, 20.0);
  t159 = t16 * t64;
  t162 = t5 * t30 * rdivide(1.0, 20.0) + t7 * t16 * t30;
  t165 = t5 * t8 * t10 + t3 * t6 * t7 * t8;
  t168 = (t16 * t165 + t3 * t5 * t6 * t8 * rdivide(1.0, 20.0)) - t7 * t8 * t10 *
    rdivide(1.0, 20.0);
  t172 = (t35 * t144 * (t44 - t78) * rdivide(24.0, 35.0) + t3 * t4 * t8 * t12 *
          rdivide(6.0, 25.0)) - t20 * t54 * rdivide(6.0, 25.0);
  t174 = t20 * t83 * rdivide(6.0, 25.0) - t22 * t35 * t65 * t83 * rdivide(6.0,
    25.0);
  t175 = t20 * t76 * rdivide(6.0, 25.0);
  t176 = t35 * t65 * t103 * rdivide(24.0, 35.0);
  t178 = t35 * t100 * t111 * rdivide(24.0, 35.0);
  t185 = t65 * t100 * t107 * t125 * rdivide(24.0, 35.0);
  t180 = t178 - t185;
  t183 = t14 * t64 * rdivide(7.0, 20.0) + t22 * t30 * rdivide(7.0, 20.0);
  t184 = t35 * t183 * (t44 - t78) * rdivide(24.0, 35.0);
  t188 = (t12 * t61 * rdivide(6.0, 25.0) + t5 * t20 * t30 * rdivide(6.0, 25.0))
    - t35 * t65 * t138 * rdivide(24.0, 35.0);
  t189 = t52 * t107 * t125 * (t73 - t135) * rdivide(24.0, 35.0);
  t190 = t35 * t130 * (t73 - t135) * rdivide(24.0, 35.0);
  t191 = t189 + t190;
  t192 = t189 - t35 * t52 * t183 * rdivide(24.0, 35.0);
  t194 = t3 * t6 * t14 * rdivide(7.0, 20.0) - t3 * t4 * t5 * t22 * rdivide(7.0,
    20.0);
  t195 = t3 * t6 * t18;
  t196 = t3 * t4 * t24;
  t199 = (t6 * t10 * t24 + t4 * t5 * t10 * t16) - t4 * t10 * t18;
  t202 = t3 * t7 + t5 * t6 * t10;
  t205 = t22 * t202 * rdivide(7.0, 20.0) + t4 * t10 * t14 * rdivide(7.0, 20.0);
  t208 = t3 * t4 * t5 * rdivide(1.0, 20.0) + t3 * t4 * t7 * t16;
  t209 = t5 * t10 * rdivide(1.0, 20.0);
  t211 = t7 * t10 - t3 * t5 * t6;
  t212 = t16 * t211;
  t215 = t5 * t10 + t3 * t6 * t7;
  t218 = t3 * t5 - t6 * t7 * t10;
  t220 = (t3 * t7 * rdivide(1.0, 20.0) + t5 * t6 * t10 * rdivide(1.0, 20.0)) -
    t16 * t218;
  t223 = t22 * t211 * rdivide(7.0, 20.0) - t3 * t4 * t14 * rdivide(7.0, 20.0);
  t225 = t35 * t111 * t223 * rdivide(24.0, 35.0);
  t226 = t107 * t125 * t223 * (t44 - t78) * rdivide(24.0, 35.0);
  t227 = -t44 + t78;
  t230 = t14 * t211 * rdivide(7.0, 20.0) + t3 * t4 * t22 * rdivide(7.0, 20.0);
  t231 = t20 * t202 * rdivide(6.0, 25.0);
  t232 = t4 * t10 * t12 * rdivide(6.0, 25.0);
  t233 = t20 * t215 * rdivide(6.0, 25.0);
  t235 = t3 * t4 * t5 * t20 * rdivide(6.0, 25.0);
  t236 = t52 * t107 * t125 * t223 * rdivide(24.0, 35.0);
  t237 = t35 * t130 * t223 * rdivide(24.0, 35.0);
  t238 = t236 + t237;
  t240 = t236 + t35 * t52 * t230 * rdivide(24.0, 35.0);
  t243 = t20 * t22 + t12 * t14;
  t244 = t5 * t35 * t243 * rdivide(24.0, 35.0);
  t246 = t124 - t179;
  t247 = rdivide(1.0, rt_powd_snf(t34, 3.0));
  t248 = t244 + t5 * t107 * t125 * t127 * rdivide(24.0, 35.0);
  t251 = t22 * t49 + t14 * t51;
  t252 = t5 * t35 * t251 * rdivide(24.0, 35.0);
  t253 = t252 - t5 * t107 * t125 * t132 * rdivide(24.0, 35.0);
  t255 = t22 * t211 - t3 * t4 * t14;
  t256 = t10 * t255;
  t257 = t11 * t14;
  t294 = t22 * t76;
  t258 = t257 - t294;
  t259 = t14 * t30;
  t295 = t22 * t64;
  t296 = t259 - t295;
  t339 = t2 * t3 * t258;
  t261 = (t256 + t3 * t8 * t296) - t339;
  t264 = t14 * t76 + t11 * t22;
  t267 = t14 * t64 + t22 * t30;
  t269 = t22 * t61 - t5 * t14 * t30;
  t271 = t22 * t27 - t5 * t11 * t14;
  t274 = t14 * t211 + t3 * t4 * t22;
  t277 = t3 * t6 * t22 + t3 * t4 * t5 * t14;
  t280 = t27 * t264;
  t281 = t61 * t267;
  t282 = t11 * t264;
  t283 = t30 * t267;
  t284 = t3 * t4 * t274;
  t287 = t14 * t43 + t2 * t3 * t4 * t22;
  t290 = t14 * t54 + t3 * t4 * t8 * t22;
  t292 = t14 * t202 - t4 * t10 * t22;
  t298 = t11 * t258;
  t299 = t30 * t296;
  t340 = t3 * t4 * t255;
  t300 = (t298 + t299) - t340;
  t302 = t27 * t258;
  t303 = t61 * t296;
  t304 = t3 * t6 * t255;
  t305 = (t302 + t303) + t304;
  t307 = t6 * t300 + t4 * t305;
  t310 = (t10 * t277 + t3 * t8 * t269) - t2 * t3 * t271;
  t311 = (t282 + t283) + t284;
  t316 = t3 * t6 * t274;
  t312 = (t280 + t281) - t316;
  t313 = ((((t282 + t283) + t284) - t27 * t271) - t61 * t269) - t3 * t6 * t277;
  t314 = t30 * t269;
  t315 = t11 * t271;
  t320 = ((((t3 * t274 + t10 * t292) + t8 * t10 * t267) - t2 * t10 * t264) - t2 *
          t3 * t287) - t3 * t8 * t290;
  t324 = ((((t11 * t287 + t4 * t10 * t274) + t2 * t3 * t4 * t264) - t30 * t290)
          - t3 * t4 * t292) - t3 * t4 * t8 * t267;
  t328 = ((((t27 * t287 + t3 * t6 * t292) + t3 * t6 * t8 * t267) - t61 * t290) -
          t6 * t10 * t274) - t2 * t3 * t6 * t264;
  t331 = (t10 * t14 * t215 + t3 * t8 * t14 * t83) - t2 * t3 * t14 * t80;
  t335 = (t14 * t27 * t80 + t14 * t61 * t83) + t3 * t6 * t14 * t215;
  t338 = (t11 * t14 * t80 + t14 * t30 * t83) - t3 * t4 * t14 * t215;
  t343 = (t10 * t274 + t2 * t3 * t264) - t3 * t8 * t267;
  t347 = t14 * t61 + t5 * t22 * t30;
  t350 = t14 * t27 + t5 * t11 * t22;
  t352 = t3 * t6 * t14 - t3 * t4 * t5 * t22;
  t355 = t22 * t43 - t2 * t3 * t4 * t14;
  t357 = t22 * t54 - t3 * t4 * t8 * t14;
  t360 = t22 * t202 + t4 * t10 * t14;
  t365 = t6 * t311 + t4 * t312;
  t368 = (t10 * t352 + t3 * t8 * t347) - t2 * t3 * t350;
  t372 = ((((-t298 - t299) + t340) + t27 * t350) + t61 * t347) + t3 * t6 * t352;
  t375 = ((((t302 + t303) + t304) + t30 * t347) + t11 * t350) - t3 * t4 * t352;
  t378 = (t10 * t22 * t215 + t3 * t8 * t22 * t83) - t2 * t3 * t22 * t80;
  t382 = (t22 * t27 * t80 + t22 * t61 * t83) + t3 * t6 * t22 * t215;
  t385 = (t11 * t22 * t80 + t22 * t30 * t83) - t3 * t4 * t22 * t215;
  t389 = ((((t3 * t255 + t10 * t360) + t2 * t10 * (t257 - t294)) - t8 * t10 *
           t296) - t2 * t3 * t355) - t3 * t8 * t357;
  t391 = (t256 - t339) + t3 * t8 * (t259 - t295);
  t395 = ((((t11 * t355 + t4 * t10 * t255) + t3 * t4 * t8 * (t259 - t295)) - t30
           * t357) - t3 * t4 * t360) - t2 * t3 * t4 * t258;
  t399 = ((((t27 * t355 + t3 * t6 * t360) + t2 * t3 * t6 * (t257 - t294)) - t61 *
           t357) - t6 * t10 * t255) - t3 * t6 * t8 * t296;
  t402 = (t10 * t343 + t3 * t4 * t311) - t3 * t6 * t312;
  t403 = t7 * t35 * t243 * rdivide(24.0, 35.0);
  t404 = t7 * t107 * t125 * t127 * rdivide(24.0, 35.0);
  t405 = t124 - t179;
  t406 = t7 * t35 * t251 * rdivide(24.0, 35.0);
  t407 = t406 - t7 * t107 * t125 * t132 * rdivide(24.0, 35.0);
  t408 = t132 * t132;
  t410 = t3 * t3;
  t411 = t27 * t80;
  t412 = t61 * t83;
  t413 = t3 * t6 * t215;
  t414 = t11 * t80;
  t415 = t30 * t83;
  t419 = ((((t3 * t215 + t10 * t218) + t2 * t10 * t80) - t2 * t3 * t89) - t8 *
          t10 * t83) - t3 * t8 * t165;
  t420 = t27 * t89;
  t421 = t3 * t6 * t218;
  t422 = t2 * t3 * t6 * t80;
  t426 = ((((t11 * t89 + t4 * t10 * t215) + t3 * t4 * t8 * t83) - t30 * t165) -
          t3 * t4 * t218) - t2 * t3 * t4 * t80;
  t427 = (t411 + t412) + t413;
  t446 = t3 * t4 * t215;
  t428 = (t414 + t415) - t446;
  t431 = (t10 * t211 + t2 * t3 * t76) - t3 * t8 * t64;
  t434 = (t27 * t76 + t61 * t64) - t3 * t6 * t211;
  t438 = (t11 * t76 + t30 * t64) + t3 * t4 * t211;
  t441 = (t2 * t3 * t7 * t11 + t3 * t4 * t7 * t10) - t3 * t7 * t8 * t30;
  t445 = ((((t411 + t412) + t413) - t7 * (t30 * t30)) - t7 * (t11 * t11)) - t7 *
    t410 * (t4 * t4);
  t447 = t7 * t30 * t61;
  t448 = t7 * t11 * t27;
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  x[4] = 0.0;
  x[5] = 0.0;
  x[6] = 0.0;
  x[7] = 0.0;
  x[8] = 0.0;
  x[9] = 0.0;
  x[10] = 0.0;
  x[11] = 0.0;
  x[12] = 0.0;
  x[13] = 0.0;
  x[14] = 0.0;
  x[15] = 0.0;
  x[16] = 0.0;
  x[17] = 0.0;
  x[18] = ((((-in1[10] * t71 + in1[11] * t95) + in1[12] * t123) - t41 * in1[9])
           - in1[8] * ((t67 - t18 * t27) + t5 * t16 * t27)) - in1[13] * t35 *
    t38 * t52 * rdivide(24.0, 35.0);
  x[19] = ((((-in1[10] * t154 - in1[11] * t162) - in1[12] * t188) - t142 * in1[9])
           + in1[8] * ((t145 - t18 * t61) + t5 * t16 * t61)) + in1[13] * t35 *
    t52 * t138 * rdivide(24.0, 35.0);
  x[20] = (((-in1[12] * ((t235 - t3 * t6 * t12 * rdivide(6.0, 25.0)) + t35 * t65
              * t194 * rdivide(24.0, 35.0)) - in1[11] * t208) + in1[8] * ((t195
              + t196) - t3 * t5 * t6 * t16)) + t199 * in1[9]) - in1[13] * t35 *
    t52 * t194 * rdivide(24.0, 35.0);
  x[21] = 0.0;
  x[22] = 0.0;
  x[23] = 0.0;
  x[24] = ((((-in1[8] * t41 + in1[10] * t59) + in1[11] * t92) + in1[12] * ((t120
              - t20 * t43 * rdivide(6.0, 25.0)) + t35 * t48 * t65 * rdivide(24.0,
              35.0))) - in1[9] * (((t68 + t16 * (t66 + t2 * t5 * t6 * t10)) - t2
             * t6 * t10 * t18) - t2 * t4 * t10 * t24)) + in1[13] * t35 * t48 *
    t52 * rdivide(24.0, 35.0);
  x[25] = ((((-in1[8] * t142 - in1[10] * t151) + in1[11] * t168) + in1[12] *
            t172) - in1[9] * (((t146 + t16 * (t72 + t5 * t6 * t8 * t10)) - t6 *
             t8 * t10 * t18) - t4 * t8 * t10 * t24)) + in1[13] * t35 * t52 *
    t144 * rdivide(24.0, 35.0);
  x[26] = (((in1[9] * (((t195 + t196) + t209) + t212) + in1[8] * t199) - in1[11]
            * t220) - in1[12] * ((t231 + t232) - t35 * t65 * t205 * rdivide(24.0,
             35.0))) + in1[13] * t35 * t52 * t205 * rdivide(24.0, 35.0);
  x[27] = (-in1[12] * ((-t244 + t35 * t227 * t248 * rdivide(24.0, 35.0)) + t5 *
                       t107 * (t246 * t246) * t247 * 0.47020408163265309) + in1
           [13] * (t35 * t52 * t248 * rdivide(24.0, 35.0) - t5 * t107 * t127 *
                   t132 * t247 * 0.47020408163265309)) + in1[11] * t7 * t35 *
    (t124 - t179) * rdivide(24.0, 35.0);
  x[28] = 0.0;
  x[29] = (in1[12] * ((-t403 + t35 * t227 * (t403 + t404) * rdivide(24.0, 35.0))
                      + t7 * t107 * t247 * (t405 * t405) * 0.47020408163265309)
           - in1[13] * (t35 * t52 * (t403 + t404) * rdivide(24.0, 35.0) - t7 *
                        t107 * t127 * t132 * t247 * 0.47020408163265309)) + in1
    [11] * t5 * t35 * (t124 - t179) * rdivide(24.0, 35.0);
  x[30] = ((((-in1[8] * t71 + in1[11] * t86) + t59 * in1[9]) + in1[12] * ((t99 +
              t101) - t12 * t30 * rdivide(6.0, 25.0))) - in1[10] * (((t67 + t68)
             + t77) - t18 * t27)) + in1[13] * t35 * t52 * t100 * rdivide(24.0,
    35.0);
  x[31] = ((((in1[10] * (((t145 - t146) + t159) - t18 * t61) - in1[8] * t154) +
             in1[11] * t158) - t151 * in1[9]) + in1[12] * ((t175 + t176) - t11 *
            t12 * rdivide(6.0, 25.0))) + in1[13] * t35 * t52 * (t102 - t104) *
    rdivide(24.0, 35.0);
  x[32] = 0.0;
  x[33] = (in1[12] * (t35 * t227 * t253 * rdivide(24.0, 35.0) - t5 * t107 * t127
                      * t132 * t247 * 0.47020408163265309) - in1[13] * ((t252 +
             t35 * t52 * t253 * rdivide(24.0, 35.0)) + t5 * t107 * t247 * t408 *
            0.47020408163265309)) + in1[11] * t7 * t35 * t132 * rdivide(24.0,
    35.0);
  x[34] = 0.0;
  x[35] = (-in1[12] * (t35 * t227 * t407 * rdivide(24.0, 35.0) - t7 * t107 *
                       t127 * t132 * t247 * 0.47020408163265309) + in1[13] *
           ((t406 + t35 * t52 * t407 * rdivide(24.0, 35.0)) + t7 * t107 * t247 *
            t408 * 0.47020408163265309)) + in1[11] * t5 * t35 * t132 * rdivide
    (24.0, 35.0);
  x[36] = ((((in1[8] * t95 + in1[10] * t86) + in1[12] * t98) + t92 * in1[9]) -
           in1[11] * ((t68 + t77) - t7 * t27 * rdivide(1.0, 20.0))) - in1[13] *
    t22 * t35 * t52 * t80 * rdivide(6.0, 25.0);
  x[37] = ((((-in1[8] * t162 + in1[10] * t158) - in1[12] * t174) + t168 * in1[9])
           - in1[11] * ((t146 - t159) + t7 * t61 * rdivide(1.0, 20.0))) + in1[13]
    * t22 * t35 * t52 * t83 * rdivide(6.0, 25.0);
  x[38] = (((-in1[8] * t208 - t220 * in1[9]) + in1[11] * ((t209 + t212) + t3 *
             t6 * t7 * rdivide(1.0, 20.0))) + in1[12] * (t233 - t22 * t35 * t65 *
            t215 * rdivide(6.0, 25.0))) - in1[13] * t22 * t35 * t52 * t215 *
    rdivide(6.0, 25.0);
  x[39] = (((in1[8] * t310 - in1[11] * t331) + t320 * in1[9]) - in1[13] * t35 *
           t52 * t261 * rdivide(24.0, 35.0)) + in1[12] * t35 * t227 * t261 *
    rdivide(24.0, 35.0);
  x[40] = (((-in1[8] * t368 - in1[11] * t378) + t389 * in1[9]) + in1[13] * t35 *
           t52 * t343 * rdivide(24.0, 35.0)) - in1[12] * t35 * t227 * t343 *
    rdivide(24.0, 35.0);
  x[41] = (in1[8] * t441 - in1[11] * t431) - t419 * in1[9];
  x[42] = ((((-in1[13] * (t35 * t52 * (t118 - t65 * t103 * t107 * t108 * rdivide
    (24.0, 35.0)) * rdivide(24.0, 35.0) - t35 * t113 * t132 * rdivide(24.0, 35.0))
              + in1[8] * t123) + in1[11] * t98) + in1[10] * ((t99 + t101) - t12 *
             t30 * rdivide(6.0, 25.0))) + in1[12] * ((((t112 - t11 * t20 *
    rdivide(6.0, 25.0)) - t12 * t76 * rdivide(6.0, 25.0)) + t35 * t113 * t127 *
             rdivide(24.0, 35.0)) + t35 * t65 * (t117 - t118) * rdivide(24.0,
             35.0))) + in1[9] * ((t120 - t20 * t43 * rdivide(6.0, 25.0)) + t35 *
    t48 * (t44 - t78) * rdivide(24.0, 35.0));
  x[43] = ((((in1[13] * (t35 * t52 * (t184 - t65 * t100 * t107 * t125 * rdivide
    (24.0, 35.0)) * rdivide(24.0, 35.0) - t35 * t132 * t180 * rdivide(24.0, 35.0))
              - in1[8] * t188) - in1[11] * t174) + t172 * in1[9]) + in1[10] *
           ((t175 + t176) - t11 * t12 * rdivide(6.0, 25.0))) + in1[12] *
    ((((-t178 + t20 * t30 * rdivide(6.0, 25.0)) + t12 * t64 * rdivide(6.0, 25.0))
      + t35 * (t44 - t78) * (t184 - t185) * rdivide(24.0, 35.0)) - t35 * t127 *
     t180 * rdivide(24.0, 35.0));
  x[44] = (((in1[12] * ((((t225 + t12 * t211 * rdivide(6.0, 25.0)) + t35 * (t124
    - t179) * (t225 - t226) * rdivide(24.0, 35.0)) + t35 * t227 * (t35 * t227 *
    t230 * rdivide(24.0, 35.0) + t107 * t125 * t223 * t227 * rdivide(24.0, 35.0))
    * rdivide(24.0, 35.0)) + t3 * t4 * t20 * rdivide(6.0, 25.0)) + in1[8] *
             ((-t235 + t3 * t6 * t12 * rdivide(6.0, 25.0)) + t35 * t194 * t227 *
              rdivide(24.0, 35.0))) + in1[13] * (t35 * t52 * (t226 + t35 * t230 *
              (t44 - t78) * rdivide(24.0, 35.0)) * rdivide(24.0, 35.0) + t35 *
             t132 * (t225 - t107 * t125 * t223 * (t44 - t78) * rdivide(24.0,
    35.0)) * rdivide(24.0, 35.0))) + in1[11] * (t233 + t22 * t35 * t215 * t227 *
            rdivide(6.0, 25.0))) - in1[9] * ((t231 + t232) + t35 * t205 * t227 *
    rdivide(24.0, 35.0));
  x[45] = (((in1[8] * (((t4 * t311 - t4 * t313) - t6 * t312) + t6 * (((((t280 +
    t281) + t314) + t315) - t3 * t6 * t274) - t3 * t4 * t277)) - in1[11] * (t4 *
              t335 + t6 * t338)) + in1[9] * (t6 * t324 + t4 * t328)) - in1[13] *
           t35 * t52 * t307 * rdivide(24.0, 35.0)) + in1[12] * t35 * t227 * t307
    * rdivide(24.0, 35.0);
  x[46] = (((-in1[8] * (((t4 * t300 - t6 * t305) + t4 * t372) + t6 * t375) -
             in1[11] * (t4 * t382 + t6 * t385)) + in1[9] * (t6 * t395 + t4 *
             t399)) - in1[13] * t35 * t52 * t365 * rdivide(24.0, 35.0)) + in1[12]
    * t35 * t227 * t365 * rdivide(24.0, 35.0);
  x[47] = (-in1[9] * (t6 * t426 + t4 * (((((t420 + t421) + t422) - t61 * t165) -
              t6 * t10 * t215) - t3 * t6 * t8 * t83)) + in1[11] * (t4 * t434 +
            t6 * t438)) + in1[8] * (((t4 * t428 - t6 * t427) + t6 * t445) - t4 *
    (((((t414 + t415) + t447) + t448) - t3 * t4 * t215) - t4 * t6 * t7 * t410));
  x[48] = ((((in1[12] * (t35 * t65 * t133 * rdivide(24.0, 35.0) - t35 * t127 *
    (t126 + t35 * t103 * t130 * rdivide(24.0, 35.0)) * rdivide(24.0, 35.0)) -
              in1[13] * ((t134 + t35 * t132 * (t126 + t134) * rdivide(24.0, 35.0))
    - t35 * t52 * t133 * rdivide(24.0, 35.0))) - in1[8] * t35 * t38 * t52 *
             rdivide(24.0, 35.0)) + t35 * t48 * t52 * in1[9] * rdivide(24.0,
             35.0)) + in1[10] * t35 * t52 * (t73 - t135) * rdivide(24.0, 35.0))
    - in1[11] * t22 * t35 * t52 * t80 * rdivide(6.0, 25.0);
  x[49] = ((((in1[13] * ((t190 - t35 * t52 * t192 * rdivide(24.0, 35.0)) + t35 *
    t132 * t191 * rdivide(24.0, 35.0)) - in1[12] * (t35 * t65 * t192 * rdivide
    (24.0, 35.0) - t35 * t191 * (t124 - t179) * rdivide(24.0, 35.0))) + in1[8] *
             t35 * t52 * t138 * rdivide(24.0, 35.0)) + t35 * t52 * t144 * in1[9]
            * rdivide(24.0, 35.0)) + in1[10] * t35 * t52 * (t102 - t104) *
           rdivide(24.0, 35.0)) + in1[11] * t22 * t35 * t52 * t83 * rdivide(6.0,
    25.0);
  x[50] = (((-in1[13] * ((t237 - t35 * t52 * t240 * rdivide(24.0, 35.0)) + t35 *
              t132 * t238 * rdivide(24.0, 35.0)) - in1[12] * (t35 * t127 * t238 *
              rdivide(24.0, 35.0) + t35 * t227 * t240 * rdivide(24.0, 35.0))) -
            in1[8] * t35 * t52 * t194 * rdivide(24.0, 35.0)) + t35 * t52 * t205 *
           in1[9] * rdivide(24.0, 35.0)) - in1[11] * t22 * t35 * t52 * t215 *
    rdivide(6.0, 25.0);
  x[51] = (((-in1[8] * ((((t10 * t310 - t3 * t4 * (((((t280 + t281) + t314) +
    t315) - t316) - t3 * t4 * t277)) + t3 * t4 * t312) + t3 * t6 * t311) - t3 *
                        t6 * t313) - in1[9] * (((((t10 * t320 + t3 * t343) - t4 *
    t10 * t311) + t6 * t10 * t312) - t3 * t4 * t324) + t3 * t6 * t328)) + in1[11]
            * ((t10 * t331 + t3 * t6 * t335) - t3 * t4 * t338)) + in1[13] * t35 *
           t52 * ((t10 * t391 + t3 * t6 * ((t302 + t303) + t304)) - t3 * t4 *
                  t300) * rdivide(24.0, 35.0)) - in1[12] * t35 * t227 * ((t10 *
    t261 - t3 * t4 * t300) + t3 * t6 * t305) * rdivide(24.0, 35.0);
  x[52] = (((in1[8] * ((((t10 * t368 + t3 * t4 * ((t302 + t303) + t304)) + t3 *
    t6 * ((t298 + t299) - t340)) + t3 * t6 * t372) - t3 * t4 * t375) - in1[9] *
             (((((t3 * t391 + t10 * t389) + t4 * t10 * t300) - t6 * t10 * t305)
               - t3 * t4 * t395) + t3 * t6 * t399)) + in1[11] * ((t10 * t378 +
              t3 * t6 * t382) - t3 * t4 * t385)) - in1[13] * t35 * t52 * t402 *
           rdivide(24.0, 35.0)) + in1[12] * t35 * t227 * t402 * rdivide(24.0,
    35.0);
  x[53] = (-in1[8] * ((((t10 * t441 - t3 * t6 * (((((t414 + t415) - t446) + t447)
    + t448) - t4 * t6 * t7 * t410)) + t3 * t4 * t427) + t3 * t6 * t428) - t3 *
                      t4 * t445) + in1[9] * (((((t10 * t419 + t3 * ((t10 * t215
    - t2 * t3 * t80) + t3 * t8 * t83)) + t3 * t6 * (((((t420 + t421) + t422) -
    t61 * t165) - t6 * t10 * t215) - t3 * t6 * t8 * t83)) - t3 * t4 * t426) + t4
             * t10 * t428) - t6 * t10 * t427)) + in1[11] * ((t10 * t431 - t3 *
    t6 * t434) + t3 * t4 * t438);
  memcpy(&A[0], &x[0], 54U * sizeof(double));
}

void J_e_dot_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void J_e_dot_fun_terminate()
{
}
