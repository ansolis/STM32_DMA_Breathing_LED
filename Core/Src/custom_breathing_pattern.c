// See the LICENSE file in the root directory of the project
//
// This file contains the code to implement a custom LED
// breathing pattern given parameters such as breathing pattern
// period, maximum brightness, brightness levels for each step
// and the number of steps within the breathing period.
//
// Note: A breathing period is the time period during which
// a breathing pattern is repeated.

#include "custom_breathing_pattern.h"
#include "main.h"
#include <stdbool.h>
#include <math.h>


#define PATTERN_PERIOD_MS       3000
#define PATTERN_MAX_BRIGHTNESS  1000
#define PATTERN_STEP_COUNT      100
#define PATTERN_MAX_STEP_COUNT  100

#define MAX_CLOCK_COUNT         0xFFFF
#define MAX_PRESCALER           0xFFFF

#define SEC_TO_MS(s)            ((s) / 1000)  // Convert seconds to milliseconds

uint16_t gBrightnessPattern[PATTERN_MAX_STEP_COUNT] = {0};
uint32_t gSystemCoreClock;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

static bool setPeriod(uint32_t patternPeriodMs, uint32_t stepCount);
static void setPattern(uint32_t patternStepCount, uint32_t maxBrightness);
static void updateDmaSettings(void);

void SetCustomBreathingPattern(void) {
  bool success = true;

  gSystemCoreClock = HAL_RCC_GetSysClockFreq();

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop(&htim1);

  success &= setPeriod(PATTERN_PERIOD_MS, PATTERN_STEP_COUNT);
  setPattern(PATTERN_STEP_COUNT, PATTERN_MAX_BRIGHTNESS);
  updateDmaSettings();

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  SetCustomBreathingPattern();
}

static bool setPeriod(uint32_t patternPeriodMs, uint32_t stepCount) {
  uint32_t timer_period = 65536.0f;

  // prescaler * timer_period = step_period_sec * clock_frequency
  // where:
  //  - prescaler is the ratio of system clock frequency to timer counter frequency;
  //  - timer_period is the number of timer ticks before the timer rolls over to 0
  //    and issues the update event triggering a DMA transfer;
  //  - step_period_sec is the duration of one pattern step in seconds;
  //  - clock_frequency is the frequency of the system clock in Hz.
  //
  // Both prescaler and timer_period are limited to 65536 and cannot be less than 1.
  // This means if the right side of equation is less than 1 or greater than
  // 65536^2 (65536 squared), then the parameters on the other side of the
  // equation need to be adjusted. The system clock frequency is configured in
  // the CubeMx code generator tool, Clock Configuration tab.  The step_period_sec
  // parameter can be adjusted depending on how many steps are chosen and how much
  // memory is available for extra steps and what the minimum number of steps is
  // required for smooth brightness transitions in a pattern. For example, if
  // the value on the right side of the equation exceeds 65536^2, either the
  // clock_frequency or the step_period_sec can be lowered.  step_period_sec can
  // be lowered by either choosing more steps in the pattern of the same duration
  // or by making the duration of the pattern shorter.

  float pattern_period_sec = (float)patternPeriodMs / 1000.0f;

  float step_period_sec = pattern_period_sec / (float)stepCount;

  // right-hand side of the equation:
  float temp = step_period_sec * (float)gSystemCoreClock;

  // Compute the prescaler assuming maximum timer period value
  timer_period = 65536;
  uint64_t prescaler = ((uint64_t)temp / timer_period) + 1;

  if (prescaler > 65536) {
      prescaler = 65536;
  }

  timer_period = (uint32_t)((step_period_sec * (float)gSystemCoreClock) / (float)prescaler);


  if (timer_period > 65536) {
    timer_period = 65536;
  }

  if (timer_period == 0){
    timer_period = 1;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim1, timer_period - 1);
  __HAL_TIM_SET_PRESCALER(&htim1, prescaler - 1);


  // Update PWM parameters to generate PWM

  // As long as the frequency is not too high, we should be ok:
  // 100MHz system clock => 1MHz timer clock => 1ms PWM period
  __HAL_TIM_SET_AUTORELOAD(&htim4, PATTERN_MAX_BRIGHTNESS);
  __HAL_TIM_SET_PRESCALER(&htim4, 100);

  return true;
}

// Generates a smooth sine wave pattern
static void setPattern(uint32_t patternStepCount, uint32_t maxBrightness) {
  float phase;  // A value between 0 and 2*PI to get a sine wave

  for (uint32_t i = 0; i < patternStepCount / 2; i++) {
    phase = ((float)M_PI * 2.0f * (float)i) / (float)patternStepCount;
    gBrightnessPattern[i] = (float)maxBrightness / 2.0f +
        (uint16_t)(sinf(phase) * (float)maxBrightness / 2.0f);
    gBrightnessPattern[i + (patternStepCount / 2)] =
        (float)maxBrightness / 2.0f -
        (uint16_t)(sinf(phase) * (float)maxBrightness / 2.0f);
  }
}

static void updateDmaSettings(void) {
  HAL_DMA_Abort(htim1.hdma[TIM_DMA_ID_UPDATE]);

  // Configure DMA transfer from memory to TIM4 CCR1 register, which
  // controls the PWM duty cycle, on TIM1 update
  HAL_DMA_Start(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)&gBrightnessPattern,
      (uint32_t)&htim4.Instance->CCR1, ARR_SIZE(gBrightnessPattern));
}
