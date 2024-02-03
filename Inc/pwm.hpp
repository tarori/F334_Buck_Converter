#pragma once

#include "utils.hpp"
#include <algorithm>

constexpr uint32_t pwm_period = 4608;
constexpr float pwm_max = 0.80f;
constexpr float vbus = 20.0f;
constexpr float dt = 10 * 1e-6;
constexpr uint32_t deadtime = 4 * 32;
constexpr float vout_max = pwm_max * vbus;

constexpr HRTIM_HandleTypeDef* hhrtim = &hhrtim1;

constexpr uint32_t pwm_avg_num = 10;
static volatile uint32_t pwm_cnt_buf[pwm_avg_num];
uint32_t duty_int[pwm_avg_num];

static inline void pwm_set_duty(float voltage, bool enable_lo)
{
    float duty = std::clamp(voltage / vbus, 0.0f, pwm_max);

    float duty_int_base = duty * pwm_period + deadtime;
    duty_int[0] = duty_int_base - 0.4f;
    duty_int[7] = duty_int_base - 0.3f;
    duty_int[4] = duty_int_base - 0.2f;
    duty_int[2] = duty_int_base - 0.1f;
    duty_int[8] = duty_int_base;
    duty_int[3] = duty_int_base + 0.1f;
    duty_int[5] = duty_int_base + 0.2f;
    duty_int[6] = duty_int_base + 0.3f;
    duty_int[9] = duty_int_base + 0.4f;
    duty_int[1] = duty_int_base + 0.5f;
    __asm("NOP");

    // HAL_HRTIM_Set_Compare(hhrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, pwm_period - duty_int);
    for (uint32_t i = 0; i < pwm_avg_num; ++i) {
        pwm_cnt_buf[i] = pwm_period - duty_int[i];
    }

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
    pwm_cnt_buf[0] = 0;
    pwm_cnt_buf[1] = 0;
    pwm_cnt_buf[2] = 0;
    pwm_cnt_buf[3] = 0;
    pwm_cnt_buf[4] = 0;
}
