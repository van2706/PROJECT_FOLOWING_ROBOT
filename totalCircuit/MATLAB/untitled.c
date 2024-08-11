/*
 * File: untitled.c
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

#include "untitled.h"
#include "rtwtypes.h"

/* Block signals (default storage) */
B_untitled_T untitled_B;

/* Block states (default storage) */
DW_untitled_T untitled_DW;

/* External inputs (root inport signals with default storage) */
ExtU_untitled_T untitled_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_untitled_T untitled_Y;

/* Real-time model */
static RT_MODEL_untitled_T untitled_M_;
RT_MODEL_untitled_T *const untitled_M = &untitled_M_;

/* Model step function */
void untitled_step(void)
{
  real_T rtb_FilterCoefficient;
  real_T rtb_Sum;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   */
  rtb_Sum = untitled_U.setpoint - untitled_U.rpm;

  /* Gain: '<S36>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter'
   *  Gain: '<S27>/Derivative Gain'
   *  Sum: '<S28>/SumD'
   */
  rtb_FilterCoefficient = (0.0 * rtb_Sum - untitled_DW.Filter_DSTATE) * 100.0;

  /* Sum: '<S42>/Sum' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   */
  untitled_B.pwm = (rtb_Sum + untitled_DW.Integrator_DSTATE) +
    rtb_FilterCoefficient;

  /* Saturate: '<Root>/Saturation' */
  if (untitled_B.pwm > 100.0) {
    /* Sum: '<S42>/Sum' incorporates:
     *  Saturate: '<Root>/Saturation'
     */
    untitled_B.pwm = 100.0;
  } else if (untitled_B.pwm < 0.0) {
    /* Sum: '<S42>/Sum' incorporates:
     *  Saturate: '<Root>/Saturation'
     */
    untitled_B.pwm = 0.0;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Outport: '<Root>/Out1' */
  untitled_Y.Out1 = untitled_B.pwm;

  /* Update for DiscreteIntegrator: '<S28>/Filter' */
  untitled_DW.Filter_DSTATE += 0.05 * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' */
  untitled_DW.Integrator_DSTATE += 0.05 * rtb_Sum;
}

/* Model initialize function */
void untitled_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void untitled_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
