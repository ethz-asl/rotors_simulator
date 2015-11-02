#ifndef __C_FUN_H__
#define __C_FUN_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "C_fun_types.h"

extern void C_fun(const double in1[17], double A[81]);
extern void C_fun_initialize();
extern void C_fun_terminate();

#endif
