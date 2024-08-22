#pragma once

#include <stdint.h>
#include <algorithm>

class PIDRegulator
{
public:
    PIDRegulator(float p_gain, float i_gain, float d_gain, float dt) : p_gain(p_gain), i_gain(i_gain), d_gain(d_gain), dt(dt)
    {
    }

    void reset()
    {
        integ = 0.0f;
    }

    float operator()(float data)
    {
        float output_p = p_gain * data;
        float output_i = i_gain * integ;
        float output_d = d_gain * (data - prev) / dt;
        float output = output_p + output_i + output_d;
        integ += data * dt;
        last_integ = data;
        prev = data;
        return output;
    }

    void revert_integ(bool positive)
    {
        if (positive && i_gain * last_integ > 0) {
            integ -= last_integ * dt;
        } else if (!positive && i_gain * last_integ < 0) {
            integ -= last_integ * dt;
        }
    }

private:
    float p_gain;
    float i_gain;
    float d_gain;
    float dt;

    float integ = 0;
    float prev = 0;
    float last_integ = 0;
};
