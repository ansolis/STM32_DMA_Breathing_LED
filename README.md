# STM32_DMA_Breathing_LED

## Description

Project using DMA on an STM32 MCU to change PWM duty cycle to control an LED
breathing pattern without using CPU

## Overview

The main logic is found in Core/Src/main.c \
The code initializes:
- TIM1 to trigger periodic DMA transfers, which update PWM duty cycle on TIM4;
- TIM4 to generate PWM output on pin PD12 connected to a green LED on the
STM32F4DISCOVERY board;
- DMA for periodic transfers of the new values from a memory array to compare
CCR1 register of TIM4 channel 1, which controls PWM duty cycle.

The values stored in the memory array describe the PWM duty cycle values for
the entire LED "breathing" pattern period.  The dimming of the LED depends on
the PWM duty cycle, where the higher duty cycle values result in higher LED
brightness and lower duty cycle values result in dimmer LED brightness.

## PWM Period and Duty Cycle
PWM period is a period of time it takes for the PWM signal to repeat. \
PWM duty cycle is the percentage of time the PWM output is "active".

To calculate PWM duty cycle, divide the value of the compare CCR1 register
(output "active" ticks) by the value of the AAR register (period ticks)
and multiply the result by 100.

The AAR register is the Auto-Reload register, which contains the PWM period
value in timer ticks.

The CCR1 register is channel 1 capture-compare register, which contains the
value at which the output of the channel (typically on an MCU pin) becomes
"active" - or "high" if the channel polarity is set to "High".

### Example 1:

AAR = 1 => PWM period = 2 ticks
CCR1 = 1 => PWM channel output switches from active to inactive when the
timer counter reaches 1.

The signals representing the outputs of the timer in PWM mode are OC1 and
OC1REF.  OC1REF is always active high, and OC1 is either the same as OC1REF
or the inverted OC1REF depending on the polarity bit CC1P. For the next
example, we'll assume the output is just OC1REF and the polarity is always
set to active high.

In this setup, PWM output OC1 will be high while the counter value is 0 and
will toggle low when the counter is incremented to 1. The OC1 will stay
low until the counter rolls over from 1 to 0. At rollover, the OC1 will go
high and stay high until the counter reaches 1 again.  The PWM duty cycle
is (active_duration / period_duration ) * 100 = (1 / 2) * 100 = 50%
```
Counter: <===0===><===1===><===0===><===1===>

OC1:     ----------        ----------
                  |        |        |
                  ----------        ---------
```

### Example 2:

AAR = 2 => PWM period = 3 ticks
CCR1 = 2 => PWM channel output switches from active to inactive when the
timer counter reaches 2.

PWM duty cycle is (2 / 3) * 100 = 66.6%

```
Counter: <===0===><===1===><===2===><===0===><===1===><===2===>

OC1:     -------------------        -------------------
                           |        |                 |
                           ----------                 ---------
```

## IDE
Project was created with STM32CubeIDE 1.11.0
