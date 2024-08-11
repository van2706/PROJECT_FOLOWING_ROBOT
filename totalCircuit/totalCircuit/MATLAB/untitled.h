/*
 * File: untitled.h
 *
 * Code generated for Simulink model 'untitled'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Wed Nov  8 23:21:20 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_untitled_h_
#define RTW_HEADER_untitled_h_
#ifndef untitled_COMMON_INCLUDES_
#define untitled_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* untitled_COMMON_INCLUDES_ */

#include "untitled_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T pwm;                          /* '<Root>/Saturation' */
} B_untitled_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Filter_DSTATE;                /* '<S28>/Filter' */
  real_T Integrator_DSTATE;            /* '<S33>/Integrator' */
} DW_untitled_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T setpoint;                     /* '<Root>/In1' */
  real_T rpm;                          /* '<Root>/In2' */
} ExtU_untitled_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_untitled_T;

/* Real-time Model Data Structure */
struct tag_RTM_untitled_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_untitled_T untitled_B;

/* Block states (default storage) */
extern DW_untitled_T untitled_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_untitled_T untitled_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_untitled_T untitled_Y;

/* Model entry point functions */
extern void untitled_initialize(void);
extern void untitled_step(void);
extern void untitled_terminate(void);

/* Real-time Model object */
extern RT_MODEL_untitled_T *const untitled_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S30>/Integral Gain' : Eliminated nontunable gain of 1
 * Block '<S38>/Proportional Gain' : Eliminated nontunable gain of 1
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
 * '<Root>' : 'untitled'
 * '<S1>'   : 'untitled/Discrete PID Controller'
 * '<S2>'   : 'untitled/Discrete PID Controller/Anti-windup'
 * '<S3>'   : 'untitled/Discrete PID Controller/D Gain'
 * '<S4>'   : 'untitled/Discrete PID Controller/Filter'
 * '<S5>'   : 'untitled/Discrete PID Controller/Filter ICs'
 * '<S6>'   : 'untitled/Discrete PID Controller/I Gain'
 * '<S7>'   : 'untitled/Discrete PID Controller/Ideal P Gain'
 * '<S8>'   : 'untitled/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'untitled/Discrete PID Controller/Integrator'
 * '<S10>'  : 'untitled/Discrete PID Controller/Integrator ICs'
 * '<S11>'  : 'untitled/Discrete PID Controller/N Copy'
 * '<S12>'  : 'untitled/Discrete PID Controller/N Gain'
 * '<S13>'  : 'untitled/Discrete PID Controller/P Copy'
 * '<S14>'  : 'untitled/Discrete PID Controller/Parallel P Gain'
 * '<S15>'  : 'untitled/Discrete PID Controller/Reset Signal'
 * '<S16>'  : 'untitled/Discrete PID Controller/Saturation'
 * '<S17>'  : 'untitled/Discrete PID Controller/Saturation Fdbk'
 * '<S18>'  : 'untitled/Discrete PID Controller/Sum'
 * '<S19>'  : 'untitled/Discrete PID Controller/Sum Fdbk'
 * '<S20>'  : 'untitled/Discrete PID Controller/Tracking Mode'
 * '<S21>'  : 'untitled/Discrete PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'untitled/Discrete PID Controller/Tsamp - Integral'
 * '<S23>'  : 'untitled/Discrete PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'untitled/Discrete PID Controller/postSat Signal'
 * '<S25>'  : 'untitled/Discrete PID Controller/preSat Signal'
 * '<S26>'  : 'untitled/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S27>'  : 'untitled/Discrete PID Controller/D Gain/Internal Parameters'
 * '<S28>'  : 'untitled/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S29>'  : 'untitled/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S30>'  : 'untitled/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S31>'  : 'untitled/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S32>'  : 'untitled/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S33>'  : 'untitled/Discrete PID Controller/Integrator/Discrete'
 * '<S34>'  : 'untitled/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S35>'  : 'untitled/Discrete PID Controller/N Copy/Disabled'
 * '<S36>'  : 'untitled/Discrete PID Controller/N Gain/Internal Parameters'
 * '<S37>'  : 'untitled/Discrete PID Controller/P Copy/Disabled'
 * '<S38>'  : 'untitled/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S39>'  : 'untitled/Discrete PID Controller/Reset Signal/Disabled'
 * '<S40>'  : 'untitled/Discrete PID Controller/Saturation/Passthrough'
 * '<S41>'  : 'untitled/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S42>'  : 'untitled/Discrete PID Controller/Sum/Sum_PID'
 * '<S43>'  : 'untitled/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S44>'  : 'untitled/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S45>'  : 'untitled/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S46>'  : 'untitled/Discrete PID Controller/Tsamp - Integral/Passthrough'
 * '<S47>'  : 'untitled/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S48>'  : 'untitled/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S49>'  : 'untitled/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_untitled_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
