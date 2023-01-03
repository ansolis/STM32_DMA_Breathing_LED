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


#define PATTERN_PERIOD_MS       4000
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
  uint32_t stepDurationMs = patternPeriodMs / stepCount;
  uint32_t prescaler = 1;
  uint32_t temp_period_ms;
  uint32_t timer_period_ticks;
  //uint32_t temp_pwm_period;

  // sys_clock_hz / prescaler = timer_clock_hz
  // period_ticks / timer_clock_hz = period_sec = period_ms / 1000
  // period_ms = 1000 * period_ticks / timer_clock_hz =
  //   = 1000 * period_ticks / (sys_clock_hz / prescaler) =
  //   = 1000 * period_ticks * prescaler / sys_clock_hz
  // period_ms = 1000 * period_ticks * prescaler / sys_clock_hz

  temp_period_ms = (uint64_t)1000 * ((uint64_t)MAX_CLOCK_COUNT * (uint64_t)prescaler) / (uint64_t)gSystemCoreClock;

  while (temp_period_ms < stepDurationMs) {
    prescaler *= 2;
    if (prescaler > MAX_PRESCALER) {
      return false;
    }
    temp_period_ms = ((uint64_t)1000 * ((uint64_t)MAX_CLOCK_COUNT * (uint64_t)prescaler) / (uint64_t)gSystemCoreClock);
  }

  timer_period_ticks = ((uint64_t)stepDurationMs * (uint64_t)gSystemCoreClock) /
      ((uint64_t)1000 * (uint64_t)prescaler);

  __HAL_TIM_SET_AUTORELOAD(&htim1, timer_period_ticks);
  __HAL_TIM_SET_PRESCALER(&htim1, prescaler);


  // Update PWM parameters
//  prescaler = 1;
//  temp_pwm_period = ((uint64_t)MAX_CLOCK_COUNT * (uint64_t)prescaler) / (uint64_t)gSystemCoreClock;
//
//  while (temp_pwm_period < PATTERN_MAX_BRIGHTNESS) {
//    prescaler *= 2;
//    if (prescaler > MAX_PRESCALER) {
//      return false;
//    }
//    temp_pwm_period = ((uint64_t)MAX_CLOCK_COUNT * (uint64_t)prescaler) / (uint64_t)gSystemCoreClock);
//  }
//
//  temp_pwm_period = ((uint64_t)PATTERN_MAX_BRIGHTNESS * (uint64_t)gSystemCoreClock) /
//        (uint64_t)prescaler;

  // As long as the frequency is not too high, we should be ok:
  // 100MHz system clock => 1MHz timer clock => 1us PWM period
  __HAL_TIM_SET_AUTORELOAD(&htim4, PATTERN_MAX_BRIGHTNESS);
  __HAL_TIM_SET_PRESCALER(&htim4, 100);

  return true;
}

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
