#ifndef __M_TRIU_FUN_H__
#define __M_TRIU_FUN_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "M_triu_fun_types.h"

extern void M_triu_fun(const double in1[8], double A[81]);
extern void M_triu_fun_initialize();
extern void M_triu_fun_terminate();

#endif
