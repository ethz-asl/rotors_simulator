#ifndef __P_EE_FUN_H__
#define __P_EE_FUN_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "p_ee_fun_types.h"

extern void p_ee_fun(const double in1[9], double A[3]);
extern void p_ee_fun_initialize();
extern void p_ee_fun_terminate();

#endif
