/*
 * arduino_buckboost.h
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

#ifndef RTW_HEADER_arduino_buckboost_h_
#define RTW_HEADER_arduino_buckboost_h_
#include <string.h>
#include <stddef.h>
#ifndef arduino_buckboost_COMMON_INCLUDES_
# define arduino_buckboost_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* arduino_buckboost_COMMON_INCLUDES_ */

#include "arduino_buckboost_types.h"
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T a00o;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a01o;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a10o;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a11o;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a00c;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a01c;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a10c;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a11c;                       /* '<Root>/DC-to-DC Converter' */
  real32_T b0o;                        /* '<Root>/DC-to-DC Converter' */
  real32_T b1o;                        /* '<Root>/DC-to-DC Converter' */
  real32_T b0c;                        /* '<Root>/DC-to-DC Converter' */
  real32_T b1c;                        /* '<Root>/DC-to-DC Converter' */
  real32_T a01d;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a10d;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a11d;                       /* '<Root>/DC-to-DC Converter' */
  real32_T a00d;                       /* '<Root>/DC-to-DC Converter' */
  real32_T b0d;                        /* '<Root>/DC-to-DC Converter' */
  real32_T b1d;                        /* '<Root>/DC-to-DC Converter' */
  uint8_T is_active_c1_arduino_buckboost;/* '<Root>/DC-to-DC Converter' */
  uint8_T is_c1_arduino_buckboost;     /* '<Root>/DC-to-DC Converter' */
} DW_arduino_buckboost_T;

/* Continuous states (default storage) */
typedef struct {
  real_T SFunction_CSTATE[3];          /* '<Root>/DC-to-DC Converter' */
} X_arduino_buckboost_T;

/* State derivatives (default storage) */
typedef struct {
  real_T SFunction_CSTATE[3];          /* '<Root>/DC-to-DC Converter' */
} XDot_arduino_buckboost_T;

/* State disabled  */
typedef struct {
  boolean_T SFunction_CSTATE[3];       /* '<Root>/DC-to-DC Converter' */
} XDis_arduino_buckboost_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Real-time Model Data Structure */
struct tag_RTM_arduino_buckboost_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_arduino_buckboost_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[3];
  real_T odeF[3][3];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Continuous states (default storage) */
extern X_arduino_buckboost_T arduino_buckboost_X;

/* Block signals and states (default storage) */
extern DW_arduino_buckboost_T arduino_buckboost_DW;

/* Model entry point functions */
extern void arduino_buckboost_initialize(void);
extern void arduino_buckboost_step(void);
extern void arduino_buckboost_terminate(void);

/* Real-time Model object */
extern RT_MODEL_arduino_buckboost_T *const arduino_buckboost_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'arduino_buckboost'
 * '<S1>'   : 'arduino_buckboost/DC-to-DC Converter'
 * '<S2>'   : 'arduino_buckboost/Digital Output'
 */
#endif                                 /* RTW_HEADER_arduino_buckboost_h_ */
