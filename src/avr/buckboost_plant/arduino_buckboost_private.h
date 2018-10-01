/*
 * arduino_buckboost_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "arduino_buckboost".
 *
 * Model version              : 1.149
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C source code generated on : Sat Sep 29 21:26:46 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_arduino_buckboost_private_h_
#define RTW_HEADER_arduino_buckboost_private_h_
#include "rtwtypes.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

/* private model entry point functions */
extern void arduino_buckboost_derivatives(void);

#endif                                 /* RTW_HEADER_arduino_buckboost_private_h_ */
