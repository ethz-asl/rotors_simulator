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
  double t259;
  double t260;
  double t261;
  double t298;
  double t262;
  double t263;
  double t299;
  double t300;
  double t343;
  double t265;
  double t268;
  double t271;
  double t274;
  double t275;
  double t276;
  double t278;
  double t280;
  double t283;
  double t284;
  double t285;
  double t288;
  double t291;
  double t294;
  double t296;
  double t302;
  double t303;
  double t344;
  double t304;
  double t306;
  double t307;
  double t308;
  double t309;
  double t311;
  double t314;
  double t318;
  double t321;
  double t324;
  double t325;
  double t330;
  double t326;
  double t327;
  double t328;
  double t329;
  double t334;
  double t338;
  double t342;
  double t347;
  double t351;
  double t354;
  double t356;
  double t359;
  double t361;
  double t364;
  double t369;
  double t372;
  double t376;
  double t379;
  double t382;
  double t386;
  double t389;
  double t393;
  double t395;
  double t399;
  double t403;
  double t406;
  double t407;
  double t408;
  double t409;
  double t410;
  double t411;
  double t412;
  double t414;
  double t415;
  double t416;
  double t417;
  double t418;
  double t419;
  double t423;
  double t424;
  double t425;
  double t426;
  double t430;
  double t431;
  double t450;
  double t432;
  double t435;
  double t438;
  double t442;
  double t445;
  double t449;
  double t451;
  double t452;
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
  t16 = t12 * rdivide(7.0, 20.0) + t14 * rdivide(2.0, 5.0);
  t18 = t7 * rdivide(1.0, 20.0) + rdivide(1.0, 10.0);
  t20 = cos(in1[4]);
  t22 = cos(in1[6]);
  t24 = (t20 * rdivide(7.0, 20.0) + t22 * rdivide(2.0, 5.0)) - rdivide(3.0, 50.0);
  t27 = t4 * t8 + t2 * t6 * t10;
  t30 = t2 * t6 + t4 * t8 * t10;
  t31 = sin(in1[7]);
  t32 = cos(in1[7]);
  t33 = t14 * t32;
  t45 = t22 * t31;
  t34 = t33 - t45;
  t35 = rdivide(1.0, t34);
  t38 = t14 * t27 * rdivide(2.0, 5.0) + t5 * t11 * t22 * rdivide(2.0, 5.0);
  t41 = (t2 * t3 * t6 * t24 + t2 * t3 * t4 * t5 * t16) - t2 * t3 * t4 * t18;
  t43 = t2 * t7 * t10 - t2 * t3 * t5 * t6;
  t44 = t12 * t32;
  t48 = t22 * t43 * rdivide(2.0, 5.0) - t2 * t3 * t4 * t14 * rdivide(2.0, 5.0);
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
  t73 = t14 * t30 * rdivide(2.0, 5.0);
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
  t98 = t20 * t80 * rdivide(7.0, 20.0) - t22 * t35 * t65 * t80 * rdivide(7.0,
    20.0);
  t99 = t20 * t64 * rdivide(7.0, 20.0);
  t135 = t22 * t64 * rdivide(2.0, 5.0);
  t100 = t73 - t135;
  t101 = t35 * t65 * t100 * rdivide(7.0, 8.0);
  t102 = t11 * t14 * rdivide(2.0, 5.0);
  t104 = t22 * t76 * rdivide(2.0, 5.0);
  t103 = t102 - t104;
  t107 = t22 * t32 + t14 * t31;
  t108 = rdivide(1.0, t34 * t34);
  t111 = t20 * t32 + t12 * t31;
  t112 = t35 * t103 * t111 * rdivide(7.0, 8.0);
  t117 = t65 * t103 * t107 * t108 * rdivide(7.0, 8.0);
  t113 = t112 - t117;
  t116 = t14 * t76 * rdivide(2.0, 5.0) + t11 * t22 * rdivide(2.0, 5.0);
  t118 = t35 * t65 * t116 * rdivide(7.0, 8.0);
  t120 = t2 * t3 * t4 * t12 * rdivide(7.0, 20.0);
  t123 = (t12 * t27 * rdivide(7.0, 20.0) + t5 * t11 * t20 * rdivide(7.0, 20.0))
    - t35 * t38 * t65 * rdivide(7.0, 8.0);
  t124 = t12 * t22;
  a = t33 - t45;
  t125 = rdivide(1.0, a * a);
  t126 = t52 * t107 * t125 * (t102 - t104) * rdivide(7.0, 8.0);
  t179 = t14 * t20;
  t127 = t124 - t179;
  t130 = t32 * t49 + t31 * t51;
  t132 = t14 * t49 - t22 * t51;
  t133 = t126 - t35 * t52 * t116 * rdivide(7.0, 8.0);
  t134 = t35 * t130 * (t102 - t104) * rdivide(7.0, 8.0);
  t138 = t14 * t61 * rdivide(2.0, 5.0) + t5 * t22 * t30 * rdivide(2.0, 5.0);
  t142 = (t3 * t6 * t8 * t24 + t3 * t4 * t5 * t8 * t16) - t3 * t4 * t8 * t18;
  t144 = t22 * t54 * rdivide(2.0, 5.0) - t3 * t4 * t8 * t14 * rdivide(2.0, 5.0);
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
  t172 = (t35 * t144 * (t44 - t78) * rdivide(7.0, 8.0) + t3 * t4 * t8 * t12 *
          rdivide(7.0, 20.0)) - t20 * t54 * rdivide(7.0, 20.0);
  t174 = t20 * t83 * rdivide(7.0, 20.0) - t22 * t35 * t65 * t83 * rdivide(7.0,
    20.0);
  t175 = t20 * t76 * rdivide(7.0, 20.0);
  t176 = t35 * t65 * t103 * rdivide(7.0, 8.0);
  t178 = t35 * t100 * t111 * rdivide(7.0, 8.0);
  t185 = t65 * t100 * t107 * t125 * rdivide(7.0, 8.0);
  t180 = t178 - t185;
  t183 = t14 * t64 * rdivide(2.0, 5.0) + t22 * t30 * rdivide(2.0, 5.0);
  t184 = t35 * t183 * (t44 - t78) * rdivide(7.0, 8.0);
  t188 = (t12 * t61 * rdivide(7.0, 20.0) + t5 * t20 * t30 * rdivide(7.0, 20.0))
    - t35 * t65 * t138 * rdivide(7.0, 8.0);
  t189 = t52 * t107 * t125 * (t73 - t135) * rdivide(7.0, 8.0);
  t190 = t35 * t130 * (t73 - t135) * rdivide(7.0, 8.0);
  t191 = t189 + t190;
  t192 = t189 - t35 * t52 * t183 * rdivide(7.0, 8.0);
  t194 = t3 * t6 * t14 * rdivide(2.0, 5.0) - t3 * t4 * t5 * t22 * rdivide(2.0,
    5.0);
  t195 = t3 * t6 * t18;
  t196 = t3 * t4 * t24;
  t199 = (t6 * t10 * t24 + t4 * t5 * t10 * t16) - t4 * t10 * t18;
  t202 = t3 * t7 + t5 * t6 * t10;
  t205 = t22 * t202 * rdivide(2.0, 5.0) + t4 * t10 * t14 * rdivide(2.0, 5.0);
  t208 = t3 * t4 * t5 * rdivide(1.0, 20.0) + t3 * t4 * t7 * t16;
  t209 = t5 * t10 * rdivide(1.0, 20.0);
  t211 = t7 * t10 - t3 * t5 * t6;
  t212 = t16 * t211;
  t215 = t5 * t10 + t3 * t6 * t7;
  t218 = t3 * t5 - t6 * t7 * t10;
  t220 = (t3 * t7 * rdivide(1.0, 20.0) + t5 * t6 * t10 * rdivide(1.0, 20.0)) -
    t16 * t218;
  t223 = t22 * t211 * rdivide(2.0, 5.0) - t3 * t4 * t14 * rdivide(2.0, 5.0);
  t225 = t35 * t111 * t223 * rdivide(7.0, 8.0);
  t226 = t107 * t125 * t223 * (t44 - t78) * rdivide(7.0, 8.0);
  t227 = -t44 + t78;
  t230 = t14 * t211 * rdivide(2.0, 5.0) + t3 * t4 * t22 * rdivide(2.0, 5.0);
  t231 = t20 * t202 * rdivide(7.0, 20.0);
  t232 = t4 * t10 * t12 * rdivide(7.0, 20.0);
  t233 = t20 * t215 * rdivide(7.0, 20.0);
  t235 = t3 * t4 * t5 * t20 * rdivide(7.0, 20.0);
  t236 = t52 * t107 * t125 * t223 * rdivide(7.0, 8.0);
  t237 = t35 * t130 * t223 * rdivide(7.0, 8.0);
  t238 = t236 + t237;
  t240 = t236 + t35 * t52 * t230 * rdivide(7.0, 8.0);
  t243 = t20 * t22 + t12 * t14;
  t244 = t5 * t35 * t243 * rdivide(7.0, 8.0);
  t246 = t124 - t179;
  t247 = rdivide(1.0, rt_powd_snf(t34, 3.0));
  t248 = t244 + t5 * t107 * t125 * t127 * rdivide(7.0, 8.0);
  t251 = t22 * t49 + t14 * t51;
  t252 = t5 * t35 * t251 * rdivide(7.0, 8.0);
  t253 = t252 - t5 * t107 * t125 * t132 * rdivide(7.0, 8.0);
  t255 = in1[6] + 3.1415926535897931 * rdivide(1.0, 6.0);
  t256 = cos(t255);
  t257 = sin(t255);
  t259 = t211 * t256 - t3 * t4 * t257;
  t260 = t10 * t259;
  t261 = t11 * t257;
  t298 = t76 * t256;
  t262 = t261 - t298;
  t263 = t30 * t257;
  t299 = t64 * t256;
  t300 = t263 - t299;
  t343 = t2 * t3 * t262;
  t265 = (t260 + t3 * t8 * t300) - t343;
  t268 = t76 * t257 + t11 * t256;
  t271 = t64 * t257 + t30 * t256;
  t274 = t211 * t257 + t3 * t4 * t256;
  t275 = t27 * t268;
  t276 = t61 * t271;
  t278 = t61 * t256 - t5 * t30 * t257;
  t280 = t27 * t256 - t5 * t11 * t257;
  t283 = t3 * t6 * t256 + t3 * t4 * t5 * t257;
  t284 = t11 * t268;
  t285 = t30 * t271;
  t288 = t3 * t4 * t274;
  t291 = t43 * t257 + t2 * t3 * t4 * t256;
  t294 = t54 * t257 + t3 * t4 * t8 * t256;
  t296 = t202 * t257 - t4 * t10 * t256;
  t302 = t11 * t262;
  t303 = t30 * t300;
  t344 = t3 * t4 * t259;
  t304 = (t302 + t303) - t344;
  t306 = t27 * t262;
  t307 = t61 * t300;
  t308 = t3 * t6 * t259;
  t309 = (t306 + t307) + t308;
  t311 = t6 * t304 + t4 * t309;
  t314 = (t10 * t215 * t257 + t3 * t8 * t83 * t257) - t2 * t3 * t80 * t257;
  t318 = (t27 * t80 * t257 + t61 * t83 * t257) + t3 * t6 * t215 * t257;
  t321 = (t11 * t80 * t257 + t30 * t83 * t257) - t3 * t4 * t215 * t257;
  t324 = (t10 * t283 + t3 * t8 * t278) - t2 * t3 * t280;
  t325 = (t284 + t285) + t288;
  t330 = t3 * t6 * t274;
  t326 = (t275 + t276) - t330;
  t327 = ((((t284 + t285) + t288) - t27 * t280) - t61 * t278) - t3 * t6 * t283;
  t328 = t30 * t278;
  t329 = t11 * t280;
  t334 = ((((t3 * t274 + t10 * t296) + t8 * t10 * t271) - t2 * t10 * t268) - t2 *
          t3 * t291) - t3 * t8 * t294;
  t338 = ((((t27 * t291 + t3 * t6 * t296) + t3 * t6 * t8 * t271) - t61 * t294) -
          t6 * t10 * t274) - t2 * t3 * t6 * t268;
  t342 = ((((t11 * t291 + t4 * t10 * t274) + t2 * t3 * t4 * t268) - t30 * t294)
          - t3 * t4 * t296) - t3 * t4 * t8 * t271;
  t347 = (t10 * t274 + t2 * t3 * t268) - t3 * t8 * t271;
  t351 = t61 * t257 + t5 * t30 * t256;
  t354 = t27 * t257 + t5 * t11 * t256;
  t356 = t3 * t6 * t257 - t3 * t4 * t5 * t256;
  t359 = t43 * t256 - t2 * t3 * t4 * t257;
  t361 = t54 * t256 - t3 * t4 * t8 * t257;
  t364 = t202 * t256 + t4 * t10 * t257;
  t369 = t6 * t325 + t4 * t326;
  t372 = (t10 * t356 + t3 * t8 * t351) - t2 * t3 * t354;
  t376 = ((((-t302 - t303) + t344) + t27 * t354) + t61 * t351) + t3 * t6 * t356;
  t379 = ((((t306 + t307) + t308) + t30 * t351) + t11 * t354) - t3 * t4 * t356;
  t382 = (t10 * t215 * t256 + t3 * t8 * t83 * t256) - t2 * t3 * t80 * t256;
  t386 = (t27 * t80 * t256 + t61 * t83 * t256) + t3 * t6 * t215 * t256;
  t389 = (t11 * t80 * t256 + t30 * t83 * t256) - t3 * t4 * t215 * t256;
  t393 = ((((t3 * t259 + t10 * t364) + t2 * t10 * (t261 - t298)) - t8 * t10 *
           t300) - t2 * t3 * t359) - t3 * t8 * t361;
  t395 = (t260 - t343) + t3 * t8 * (t263 - t299);
  t399 = ((((t27 * t359 + t3 * t6 * t364) + t2 * t3 * t6 * (t261 - t298)) - t61 *
           t361) - t6 * t10 * t259) - t3 * t6 * t8 * t300;
  t403 = ((((t11 * t359 + t4 * t10 * t259) + t3 * t4 * t8 * (t263 - t299)) - t30
           * t361) - t3 * t4 * t364) - t2 * t3 * t4 * t262;
  t406 = (t10 * t347 + t3 * t4 * t325) - t3 * t6 * t326;
  t407 = t7 * t35 * t243 * rdivide(7.0, 8.0);
  t408 = t7 * t107 * t125 * t127 * rdivide(7.0, 8.0);
  t409 = t124 - t179;
  t410 = t7 * t35 * t251 * rdivide(7.0, 8.0);
  t411 = t410 - t7 * t107 * t125 * t132 * rdivide(7.0, 8.0);
  t412 = t132 * t132;
  t414 = t3 * t3;
  t415 = t27 * t80;
  t416 = t61 * t83;
  t417 = t3 * t6 * t215;
  t418 = t11 * t80;
  t419 = t30 * t83;
  t423 = ((((t3 * t215 + t10 * t218) + t2 * t10 * t80) - t2 * t3 * t89) - t8 *
          t10 * t83) - t3 * t8 * t165;
  t424 = t27 * t89;
  t425 = t3 * t6 * t218;
  t426 = t2 * t3 * t6 * t80;
  t430 = ((((t11 * t89 + t4 * t10 * t215) + t3 * t4 * t8 * t83) - t30 * t165) -
          t3 * t4 * t218) - t2 * t3 * t4 * t80;
  t431 = (t415 + t416) + t417;
  t450 = t3 * t4 * t215;
  t432 = (t418 + t419) - t450;
  t435 = (t10 * t211 + t2 * t3 * t76) - t3 * t8 * t64;
  t438 = (t27 * t76 + t61 * t64) - t3 * t6 * t211;
  t442 = (t11 * t76 + t30 * t64) + t3 * t4 * t211;
  t445 = (t2 * t3 * t7 * t11 + t3 * t4 * t7 * t10) - t3 * t7 * t8 * t30;
  t449 = ((((t415 + t416) + t417) - t7 * (t30 * t30)) - t7 * (t11 * t11)) - t7 *
    t414 * (t4 * t4);
  t451 = t7 * t30 * t61;
  t452 = t7 * t11 * t27;
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
    t38 * t52 * rdivide(7.0, 8.0);
  x[19] = ((((-in1[10] * t154 - in1[11] * t162) - in1[12] * t188) - t142 * in1[9])
           + in1[8] * ((t145 - t18 * t61) + t5 * t16 * t61)) + in1[13] * t35 *
    t52 * t138 * rdivide(7.0, 8.0);
  x[20] = (((-in1[12] * ((t235 - t3 * t6 * t12 * rdivide(7.0, 20.0)) + t35 * t65
              * t194 * rdivide(7.0, 8.0)) - in1[11] * t208) + in1[8] * ((t195 +
              t196) - t3 * t5 * t6 * t16)) + t199 * in1[9]) - in1[13] * t35 *
    t52 * t194 * rdivide(7.0, 8.0);
  x[21] = 0.0;
  x[22] = 0.0;
  x[23] = 0.0;
  x[24] = ((((-in1[8] * t41 + in1[10] * t59) + in1[11] * t92) + in1[12] * ((t120
              - t20 * t43 * rdivide(7.0, 20.0)) + t35 * t48 * t65 * rdivide(7.0,
              8.0))) - in1[9] * (((t68 + t16 * (t66 + t2 * t5 * t6 * t10)) - t2 *
             t6 * t10 * t18) - t2 * t4 * t10 * t24)) + in1[13] * t35 * t48 * t52
    * rdivide(7.0, 8.0);
  x[25] = ((((-in1[8] * t142 - in1[10] * t151) + in1[11] * t168) + in1[12] *
            t172) - in1[9] * (((t146 + t16 * (t72 + t5 * t6 * t8 * t10)) - t6 *
             t8 * t10 * t18) - t4 * t8 * t10 * t24)) + in1[13] * t35 * t52 *
    t144 * rdivide(7.0, 8.0);
  x[26] = (((in1[9] * (((t195 + t196) + t209) + t212) + in1[8] * t199) - in1[11]
            * t220) - in1[12] * ((t231 + t232) - t35 * t65 * t205 * rdivide(7.0,
             8.0))) + in1[13] * t35 * t52 * t205 * rdivide(7.0, 8.0);
  x[27] = (-in1[12] * ((-t244 + t35 * t227 * t248 * rdivide(7.0, 8.0)) + t5 *
                       t107 * (t246 * t246) * t247 * rdivide(49.0, 64.0)) + in1
           [13] * (t35 * t52 * t248 * rdivide(7.0, 8.0) - t5 * t107 * t127 *
                   t132 * t247 * rdivide(49.0, 64.0))) + in1[11] * t7 * t35 *
    (t124 - t179) * rdivide(7.0, 8.0);
  x[28] = 0.0;
  x[29] = (in1[12] * ((-t407 + t35 * t227 * (t407 + t408) * rdivide(7.0, 8.0)) +
                      t7 * t107 * t247 * (t409 * t409) * rdivide(49.0, 64.0)) -
           in1[13] * (t35 * t52 * (t407 + t408) * rdivide(7.0, 8.0) - t7 * t107 *
                      t127 * t132 * t247 * rdivide(49.0, 64.0))) + in1[11] * t5 *
    t35 * (t124 - t179) * rdivide(7.0, 8.0);
  x[30] = ((((-in1[8] * t71 + in1[11] * t86) + t59 * in1[9]) + in1[12] * ((t99 +
              t101) - t12 * t30 * rdivide(7.0, 20.0))) - in1[10] * (((t67 + t68)
             + t77) - t18 * t27)) + in1[13] * t35 * t52 * t100 * rdivide(7.0,
    8.0);
  x[31] = ((((in1[10] * (((t145 - t146) + t159) - t18 * t61) - in1[8] * t154) +
             in1[11] * t158) - t151 * in1[9]) + in1[12] * ((t175 + t176) - t11 *
            t12 * rdivide(7.0, 20.0))) + in1[13] * t35 * t52 * (t102 - t104) *
    rdivide(7.0, 8.0);
  x[32] = 0.0;
  x[33] = (in1[12] * (t35 * t227 * t253 * rdivide(7.0, 8.0) - t5 * t107 * t127 *
                      t132 * t247 * rdivide(49.0, 64.0)) - in1[13] * ((t252 +
             t35 * t52 * t253 * rdivide(7.0, 8.0)) + t5 * t107 * t247 * t412 *
            rdivide(49.0, 64.0))) + in1[11] * t7 * t35 * t132 * rdivide(7.0, 8.0);
  x[34] = 0.0;
  x[35] = (-in1[12] * (t35 * t227 * t411 * rdivide(7.0, 8.0) - t7 * t107 * t127 *
                       t132 * t247 * rdivide(49.0, 64.0)) + in1[13] * ((t410 +
             t35 * t52 * t411 * rdivide(7.0, 8.0)) + t7 * t107 * t247 * t412 *
            rdivide(49.0, 64.0))) + in1[11] * t5 * t35 * t132 * rdivide(7.0, 8.0);
  x[36] = ((((in1[8] * t95 + in1[10] * t86) + in1[12] * t98) + t92 * in1[9]) -
           in1[11] * ((t68 + t77) - t7 * t27 * rdivide(1.0, 20.0))) - in1[13] *
    t22 * t35 * t52 * t80 * rdivide(7.0, 20.0);
  x[37] = ((((-in1[8] * t162 + in1[10] * t158) - in1[12] * t174) + t168 * in1[9])
           - in1[11] * ((t146 - t159) + t7 * t61 * rdivide(1.0, 20.0))) + in1[13]
    * t22 * t35 * t52 * t83 * rdivide(7.0, 20.0);
  x[38] = (((-in1[8] * t208 - t220 * in1[9]) + in1[11] * ((t209 + t212) + t3 *
             t6 * t7 * rdivide(1.0, 20.0))) + in1[12] * (t233 - t22 * t35 * t65 *
            t215 * rdivide(7.0, 20.0))) - in1[13] * t22 * t35 * t52 * t215 *
    rdivide(7.0, 20.0);
  x[39] = (((in1[8] * t324 - in1[11] * t314) + t334 * in1[9]) - in1[13] * t35 *
           t52 * t265 * rdivide(7.0, 8.0)) + in1[12] * t35 * t227 * t265 *
    rdivide(7.0, 8.0);
  x[40] = (((-in1[8] * t372 - in1[11] * t382) + t393 * in1[9]) + in1[13] * t35 *
           t52 * t347 * rdivide(7.0, 8.0)) - in1[12] * t35 * t227 * t347 *
    rdivide(7.0, 8.0);
  x[41] = (in1[8] * t445 - in1[11] * t435) - t423 * in1[9];
  x[42] = ((((-in1[13] * (t35 * t52 * (t118 - t65 * t103 * t107 * t108 * rdivide
    (7.0, 8.0)) * rdivide(7.0, 8.0) - t35 * t113 * t132 * rdivide(7.0, 8.0)) +
              in1[8] * t123) + in1[11] * t98) + in1[10] * ((t99 + t101) - t12 *
             t30 * rdivide(7.0, 20.0))) + in1[12] * ((((t112 - t11 * t20 *
    rdivide(7.0, 20.0)) - t12 * t76 * rdivide(7.0, 20.0)) + t35 * t113 * t127 *
             rdivide(7.0, 8.0)) + t35 * t65 * (t117 - t118) * rdivide(7.0, 8.0)))
    + in1[9] * ((t120 - t20 * t43 * rdivide(7.0, 20.0)) + t35 * t48 * (t44 - t78)
                * rdivide(7.0, 8.0));
  x[43] = ((((in1[13] * (t35 * t52 * (t184 - t65 * t100 * t107 * t125 * rdivide
    (7.0, 8.0)) * rdivide(7.0, 8.0) - t35 * t132 * t180 * rdivide(7.0, 8.0)) -
              in1[8] * t188) - in1[11] * t174) + t172 * in1[9]) + in1[10] *
           ((t175 + t176) - t11 * t12 * rdivide(7.0, 20.0))) + in1[12] *
    ((((-t178 + t20 * t30 * rdivide(7.0, 20.0)) + t12 * t64 * rdivide(7.0, 20.0))
      + t35 * (t44 - t78) * (t184 - t185) * rdivide(7.0, 8.0)) - t35 * t127 *
     t180 * rdivide(7.0, 8.0));
  x[44] = (((in1[12] * ((((t225 + t12 * t211 * rdivide(7.0, 20.0)) + t35 * (t124
    - t179) * (t225 - t226) * rdivide(7.0, 8.0)) + t35 * t227 * (t35 * t227 *
    t230 * rdivide(7.0, 8.0) + t107 * t125 * t223 * t227 * rdivide(7.0, 8.0)) *
    rdivide(7.0, 8.0)) + t3 * t4 * t20 * rdivide(7.0, 20.0)) + in1[8] * ((-t235
    + t3 * t6 * t12 * rdivide(7.0, 20.0)) + t35 * t194 * t227 * rdivide(7.0, 8.0)))
            + in1[13] * (t35 * t52 * (t226 + t35 * t230 * (t44 - t78) * rdivide
              (7.0, 8.0)) * rdivide(7.0, 8.0) + t35 * t132 * (t225 - t107 * t125
              * t223 * (t44 - t78) * rdivide(7.0, 8.0)) * rdivide(7.0, 8.0))) +
           in1[11] * (t233 + t22 * t35 * t215 * t227 * rdivide(7.0, 20.0))) -
    in1[9] * ((t231 + t232) + t35 * t205 * t227 * rdivide(7.0, 8.0));
  x[45] = (((in1[8] * (((t4 * t325 - t4 * t327) - t6 * t326) + t6 * (((((t275 +
    t276) + t328) + t329) - t3 * t6 * t274) - t3 * t4 * t283)) - in1[11] * (t4 *
              t318 + t6 * t321)) + in1[9] * (t4 * t338 + t6 * t342)) - in1[13] *
           t35 * t52 * t311 * rdivide(7.0, 8.0)) + in1[12] * t35 * t227 * t311 *
    rdivide(7.0, 8.0);
  x[46] = (((-in1[8] * (((t4 * t304 - t6 * t309) + t4 * t376) + t6 * t379) -
             in1[11] * (t4 * t386 + t6 * t389)) + in1[9] * (t4 * t399 + t6 *
             t403)) - in1[13] * t35 * t52 * t369 * rdivide(7.0, 8.0)) + in1[12] *
    t35 * t227 * t369 * rdivide(7.0, 8.0);
  x[47] = (-in1[9] * (t6 * t430 + t4 * (((((t424 + t425) + t426) - t61 * t165) -
              t6 * t10 * t215) - t3 * t6 * t8 * t83)) + in1[11] * (t4 * t438 +
            t6 * t442)) + in1[8] * (((t4 * t432 - t6 * t431) + t6 * t449) - t4 *
    (((((t418 + t419) + t451) + t452) - t3 * t4 * t215) - t4 * t6 * t7 * t414));
  x[48] = ((((in1[12] * (t35 * t65 * t133 * rdivide(7.0, 8.0) - t35 * t127 *
    (t126 + t35 * t103 * t130 * rdivide(7.0, 8.0)) * rdivide(7.0, 8.0)) - in1[13]
              * ((t134 + t35 * t132 * (t126 + t134) * rdivide(7.0, 8.0)) - t35 *
                 t52 * t133 * rdivide(7.0, 8.0))) - in1[8] * t35 * t38 * t52 *
             rdivide(7.0, 8.0)) + t35 * t48 * t52 * in1[9] * rdivide(7.0, 8.0))
           + in1[10] * t35 * t52 * (t73 - t135) * rdivide(7.0, 8.0)) - in1[11] *
    t22 * t35 * t52 * t80 * rdivide(7.0, 20.0);
  x[49] = ((((in1[13] * ((t190 - t35 * t52 * t192 * rdivide(7.0, 8.0)) + t35 *
    t132 * t191 * rdivide(7.0, 8.0)) - in1[12] * (t35 * t65 * t192 * rdivide(7.0,
    8.0) - t35 * t191 * (t124 - t179) * rdivide(7.0, 8.0))) + in1[8] * t35 * t52
             * t138 * rdivide(7.0, 8.0)) + t35 * t52 * t144 * in1[9] * rdivide
            (7.0, 8.0)) + in1[10] * t35 * t52 * (t102 - t104) * rdivide(7.0, 8.0))
    + in1[11] * t22 * t35 * t52 * t83 * rdivide(7.0, 20.0);
  x[50] = (((-in1[13] * ((t237 - t35 * t52 * t240 * rdivide(7.0, 8.0)) + t35 *
              t132 * t238 * rdivide(7.0, 8.0)) - in1[12] * (t35 * t127 * t238 *
              rdivide(7.0, 8.0) + t35 * t227 * t240 * rdivide(7.0, 8.0))) - in1
            [8] * t35 * t52 * t194 * rdivide(7.0, 8.0)) + t35 * t52 * t205 *
           in1[9] * rdivide(7.0, 8.0)) - in1[11] * t22 * t35 * t52 * t215 *
    rdivide(7.0, 20.0);
  x[51] = (((-in1[8] * ((((t10 * t324 - t3 * t4 * (((((t275 + t276) + t328) +
    t329) - t330) - t3 * t4 * t283)) + t3 * t4 * t326) + t3 * t6 * t325) - t3 *
                        t6 * t327) - in1[9] * (((((t10 * t334 + t3 * t347) - t4 *
    t10 * t325) + t6 * t10 * t326) + t3 * t6 * t338) - t3 * t4 * t342)) + in1[11]
            * ((t10 * t314 + t3 * t6 * t318) - t3 * t4 * t321)) + in1[13] * t35 *
           t52 * ((t10 * t395 + t3 * t6 * ((t306 + t307) + t308)) - t3 * t4 *
                  t304) * rdivide(7.0, 8.0)) - in1[12] * t35 * t227 * ((t10 *
    t265 - t3 * t4 * t304) + t3 * t6 * t309) * rdivide(7.0, 8.0);
  x[52] = (((in1[8] * ((((t10 * t372 + t3 * t4 * ((t306 + t307) + t308)) + t3 *
    t6 * ((t302 + t303) - t344)) + t3 * t6 * t376) - t3 * t4 * t379) - in1[9] *
             (((((t3 * t395 + t10 * t393) + t4 * t10 * t304) - t6 * t10 * t309)
               + t3 * t6 * t399) - t3 * t4 * t403)) + in1[11] * ((t10 * t382 +
              t3 * t6 * t386) - t3 * t4 * t389)) - in1[13] * t35 * t52 * t406 *
           rdivide(7.0, 8.0)) + in1[12] * t35 * t227 * t406 * rdivide(7.0, 8.0);
  x[53] = (-in1[8] * ((((t10 * t445 - t3 * t6 * (((((t418 + t419) - t450) + t451)
    + t452) - t4 * t6 * t7 * t414)) + t3 * t4 * t431) + t3 * t6 * t432) - t3 *
                      t4 * t449) + in1[9] * (((((t10 * t423 + t3 * ((t10 * t215
    - t2 * t3 * t80) + t3 * t8 * t83)) + t3 * t6 * (((((t424 + t425) + t426) -
    t61 * t165) - t6 * t10 * t215) - t3 * t6 * t8 * t83)) - t3 * t4 * t430) + t4
             * t10 * t432) - t6 * t10 * t431)) + in1[11] * ((t10 * t435 - t3 *
    t6 * t438) + t3 * t4 * t442);
  memcpy(&A[0], &x[0], 54U * sizeof(double));
}

void J_e_dot_fun_initialize()
{
  rt_InitInfAndNaN(8U);
}

void J_e_dot_fun_terminate()
{
}

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
  t17 = sin(in1[7]) * 0.35 + sin(in1[8]) * 0.4;
  t19 = t2 * t3 - t4 * t6 * t7;
  t20 = t10 * 0.05;
  t26 = (cos(in1[7]) * 0.35 + cos(in1[8]) * 0.4) - 0.06;
  A[0] = (((in1[0] + t26 * (t4 * t6 - t2 * t3 * t7)) + t17 * (t9 * t12 + t2 *
            t10 * t11)) - t9 * (t20 + 0.1)) + t2 * t11 * t12 * 0.05;
  A[1] = (((in1[1] - t26 * (t2 * t6 + t3 * t4 * t7)) - t17 * (t12 * t19 - t4 *
            t10 * t11)) + t19 * (t20 + 0.1)) + t4 * t11 * t12 * 0.05;
  A[2] = (((in1[2] - t17 * (t7 * t10 - t6 * t11 * t12)) - t7 * t12 * 0.05) - t6 *
          t11 * (t20 + 0.1)) - t3 * t11 * t26;
}
