#include "main_cycle.hpp"
#include "utils.hpp"
#include "hrtim.h"
#include "lcd.hpp"
#include "adc.h"
#include "dac.h"
#include "i2c.h"
#include "clock.hpp"
#include <stm32f3xx.h>

#include "pwm.hpp"
#include "pid_regulator.hpp"
#include "type3_regulator.hpp"

extern bool callback_start;

/*
wz1=0.75/sqrt(LoutCout)
wz2=1/sqrt(LoutCout)
wp0=min(wz1,wp2)
wp2=1/(Cout*ESR)
wp3=2PI*fsw/2
*/

TYPE3Regulator v_regulator(10714, 14285, 10714, 100000, 314159, dt, 0, vout_max);
TYPE3Regulator i_regulator(10714, 14285, 10714, 100000, 314159, dt, 0, vout_max);

alignas(4) uint16_t adc1_buf[2];
alignas(4) uint16_t adc2_buf[2];

LCD lcd(&hi2c1);

namespace Control
{
volatile float actual_voltage;
volatile float actual_current;
volatile float actual_voltage_filtered = 0.0f;
volatile float actual_current_filtered = 0.0f;
volatile float target_voltage = 0.0f;
volatile float target_current = 0.0f;
volatile float dac_voltage = 0.0f;
volatile float target_dac_voltage = 0.0f;
volatile float output_v;
volatile float output_i;
volatile float output;
volatile bool emergency_occured = false;
}  // namespace Control

void main_loop()
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);

    hhrtim1.TimerParam[HRTIM_TIMERINDEX_TIMER_A].DMASrcAddress = (uint32_t)((void*)pwm_cnt_buf);
    hhrtim1.TimerParam[HRTIM_TIMERINDEX_TIMER_A].DMADstAddress = (uint32_t)((void*)&hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR);
    hhrtim1.TimerParam[HRTIM_TIMERINDEX_TIMER_A].DMASize = pwm_avg_num;

    HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);
    __disable_irq();
    HAL_HRTIM_WaveformCountStart_DMA(&hhrtim1, HRTIM_TIMERID_TIMER_A);
    hhrtim1.hdmaTimerA->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
    __enable_irq();

    __disable_irq();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, sizeof(adc1_buf) / sizeof(uint16_t));
    hadc1.DMA_Handle->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, sizeof(adc2_buf) / sizeof(uint16_t));
    hadc2.DMA_Handle->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
    __enable_irq();

    printf("Hello, I am working at %ldMHz\n", SystemCoreClock / 1000 / 1000);
    lcd.init();
    lcd.setBacklight(1);
    lcd.printf("Hello\nWorking at %ldMHz\n", SystemCoreClock / 1000 / 1000);
    delay_ms(1000);

    callback_start = true;

    while (1) {
        if (Control::emergency_occured) {
            printf("Emergency\n");
            continue;
        }

        // printf("%.3f V %.3f V\n", Control::dac_voltage, Control::actual_voltage);
        printf("%.5f V  %.4f A\n", Control::actual_voltage_filtered, Control::actual_current_filtered);
        lcd.locate(0, 0);
        lcd.printf("T:%6.2fV %5.2fA", Control::target_voltage, Control::target_current);
        lcd.locate(0, 1);
        lcd.printf("A:%6.2fV %5.2fA", Control::actual_voltage_filtered, Control::actual_current_filtered);
        delay_ms(250);
    }
}

constexpr float voltage_step = 0.25f;
constexpr float current_step = 0.1f;

void callback_10ms()
{
    float voltage_volume = adc2_buf[1] / 4095.0f * 20.0f;
    float current_volume = adc1_buf[1] / 4095.0f * 6.1f;

    if (voltage_volume > Control::target_voltage + voltage_step) {
        Control::target_voltage += voltage_step;
    }
    if (voltage_volume < Control::target_voltage - voltage_step) {
        Control::target_voltage -= voltage_step;
    }


    if (current_volume > Control::target_current + current_step) {
        Control::target_current += current_step;
    }
    if (current_volume < Control::target_current - current_step) {
        Control::target_current -= current_step;
    }
}

constexpr float vref = 3.30f;
constexpr float voltage_div = 11.0f / 1.0f;
constexpr float voltage_gain = 11.0f;
constexpr float shunt_resistance = 0.92f * 0.020f;
constexpr float current_mul = vref / 4095 / (5.399f / 0.399f) / shunt_resistance;
constexpr float current_offset = 744.0f;
constexpr float wire_resistance = 0.0f;

constexpr float emergency_voltage = 24.0f;
constexpr float emergency_current = 100.0f;
constexpr float filter_tau = 0.01f;

__attribute__((long_call, section(".ccmram"))) void callback_10us()
{
    if (Control::emergency_occured) {
        return;
    }

    Control::target_dac_voltage = ((voltage_gain / voltage_div * Control::target_voltage) + vref / 2) / (voltage_gain + 1);
    Control::dac_voltage += (Control::target_dac_voltage - Control::dac_voltage) * 0.25f;
    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, Control::dac_voltage / vref * 4095.0f);

    Control::actual_voltage = ((1 + voltage_gain) * Control::target_dac_voltage - adc2_buf[0] * vref / 4095.0f) / voltage_gain * voltage_div;

    Control::actual_current = (adc1_buf[0] - current_offset) * current_mul;

    Control::actual_voltage -= wire_resistance * Control::actual_current_filtered;

    Control::actual_voltage_filtered += filter_tau * (Control::actual_voltage - Control::actual_voltage_filtered);
    Control::actual_current_filtered += filter_tau * (Control::actual_current - Control::actual_current_filtered);

    if (Control::actual_current > emergency_current || Control::actual_voltage > emergency_voltage) {
        pwm_free();
        Control::emergency_occured = true;
        return;
    }

    Control::output_v = v_regulator(Control::target_voltage - Control::actual_voltage);
    Control::output_i = i_regulator(0.1f * (Control::target_current - Control::actual_current));
    Control::output = std::min(Control::output_v, Control::output_i);
    v_regulator.set_actual_output(Control::output);
    i_regulator.set_actual_output(Control::output);
    pwm_set_duty(Control::output, true);
    // pwm_set_duty(10.0f, true);
}
