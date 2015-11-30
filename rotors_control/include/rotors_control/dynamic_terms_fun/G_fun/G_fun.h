#ifndef __G_FUN_H__
#define __G_FUN_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "G_fun_types.h"

extern void G_fun(const double in1[7], double A[9]);
extern void G_fun_initialize();
extern void G_fun_terminate();

#endif
