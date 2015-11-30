#ifndef __J_E_FUN_H__
#define __J_E_FUN_H__
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "J_e_fun_types.h"

extern void J_e_fun(const double in1[8], double A[54]);
extern void J_e_fun_initialize();
extern void J_e_fun_terminate();

#endif
