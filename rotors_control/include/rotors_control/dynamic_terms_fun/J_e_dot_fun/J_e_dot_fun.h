#ifndef __J_E_DOT_FUN_H__
#define __J_E_DOT_FUN_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "J_e_dot_fun_types.h"

extern void J_e_dot_fun(const double in1[14], double A[54]);
extern void J_e_dot_fun_initialize();
extern void J_e_dot_fun_terminate();
extern void p_ee_fun(const double in1[9], double A[3]);

#endif
