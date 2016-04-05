#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <cstddef>
#include <cmath>

template <size_t WIDTH>
class LowpassFilter
{
    public:
        LowpassFilter();
        ~LowpassFilter();
        void SetCutFrequency(double frequency, double sampleRate);
        void Initialize();
        void DoFilter(const double (&signalIn)[WIDTH], double (&signalOut)[WIDTH]);

    private:
        double wcut;
        double dt;
        double yddot[WIDTH];
        double ydot[WIDTH];
        double y[WIDTH];
        double coe1;
        double coe2;
};

template <size_t WIDTH>
LowpassFilter<WIDTH>::LowpassFilter()
{
}

template <size_t WIDTH>
LowpassFilter<WIDTH>::~LowpassFilter()
{
}

template <size_t WIDTH>
void LowpassFilter<WIDTH>::SetCutFrequency(double frequency, double sampleRate)
{
    if (frequency < 1e-3)
    {
        frequency = 1;
    }
    if ( sampleRate < 1e-3 )
    {
        sampleRate = 1000;
    }

    wcut = 2 * 3.1415926535897931 * frequency;
    dt = 1.0 / sampleRate;

    coe1 = sqrt(2) / wcut;
    coe2 = 1.0 / wcut /wcut;
}

template <size_t WIDTH>
void LowpassFilter<WIDTH>::DoFilter(const double (&signalIn)[WIDTH], double (&signalOut)[WIDTH])
{
    for (int i = 0; i < WIDTH; ++i)
    {
        yddot[i]  = coe2 * signalIn[i] - coe1 * ydot[i] - coe2 * y[i];
        ydot[i]  += yddot[i] * dt;
        y[i]     += ydot[i] * dt;
        signalOut[i] = y[i];
    }
}

template <size_t WIDTH>
void LowpassFilter<WIDTH>::Initialize()
{
    for (int i = 0; i < WIDTH; ++i)
    {
        yddot[i] = 0; 
        ydot[i]  = 0;
        y[i]     = 0;
    }
}
#endif
