#pragma once

#include "utils.hpp"
#include <algorithm>

constexpr uint32_t pwm_period = 9216;
constexpr float pwm_max = 0.90f;
constexpr float vbus = 20.0f;
constexpr float dt = 10 * 1e-6;
constexpr uint32_t deadtime = 2 * 32;
constexpr uint32_t minimum_duty = 96;
constexpr float vout_max = pwm_max * vbus;

constexpr HRTIM_HandleTypeDef* hhrtim = &hhrtim1;

constexpr uint32_t pwm_avg_num = 4;
inline volatile uint32_t pwm_cnt_buf[pwm_avg_num];
uint32_t duty_int[pwm_avg_num];

static inline void pwm_set_duty(float voltage, bool enable_lo)
{
    float duty = std::clamp(voltage / vbus, 0.0f, pwm_max);

    float duty_int_base = duty * pwm_period + deadtime;
    if (duty_int_base < minimum_duty) {
        duty_int_base = 96;
    }

    duty_int[0] = duty_int_base + 0.0f;
    duty_int[1] = duty_int_base + 0.25f;
    duty_int[2] = duty_int_base + 0.5f;
    duty_int[3] = duty_int_base + 0.75f;

    __asm("NOP");  // Prevent other processing in critical section

    // HAL_HRTIM_Set_Compare(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, duty_int);
    for (uint32_t i = 0; i < pwm_avg_num; ++i) {
        pwm_cnt_buf[i] = duty_int[i];
    }

    __asm("NOP");

    if (enable_lo) {
        HAL_HRTIM_Enable_Output(hhrtim, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    } else {
        HAL_HRTIM_Enable_Output(hhrtim, HRTIM_OUTPUT_TA2);
        HAL_HRTIM_Disable_Output(hhrtim, HRTIM_OUTPUT_TA1);
    }
}

static inline void pwm_free()
{
    HAL_HRTIM_Disable_Output(hhrtim, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
    // HAL_HRTIM_Set_Compare(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, 0);
}
