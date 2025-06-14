#include "filter.h"

#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

 
// PT1 low-pass filter
// Get filter gain for pt1 (cutoff frequency, sampling time)
float pt1FilterGain(float f_cut, float dT)
{
    float RC = 1 / (2 * M_PIf * f_cut);
    return dT / (RC + dT);
}

// Initialize pt1 low-pass filter
void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

// Update filter gain
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

// Apply pt1 low-pass filter
float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}


// Get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1);F2 = f02/ f1
// (center frequency, cutoff frequency)
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

// Initialize second-order low-pass filter
void biquadFilterInitLPF(biquadFilter_t *filter, unsigned int filterFreq ,unsigned int samplingFreq)
{
    biquadFilterInit(filter, filterFreq, samplingFreq,  BIQUAD_Q, FILTER_LPF);
}


void biquadFilterInit(biquadFilter_t *filter, float filterFreq, unsigned int refreshRate, float Q, biquadFilterType_e filterType)
{
    biquadFilterUpdate(filter, (unsigned int)filterFreq, (unsigned int)refreshRate, Q, filterType);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}




// Initialize second-order filter
void biquadFilterUpdate(biquadFilter_t *filter, unsigned int filterFreq, unsigned int refreshRate, float Q, biquadFilterType_e filterType)
{
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * refreshRate * 0.000001f;
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2.0f * Q);

    switch (filterType) {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        filter->b1 = 1 - cs;
        filter->b0 = filter->b1 * 0.5f;
        filter->b2 = filter->b0;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        filter->b0 = 1;
        filter->b1 = -2 * cs;
        filter->b2 = 1;
        filter->a1 = filter->b1;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_BPF:
        filter->b0 = alpha;
        filter->b1 = 0;
        filter->b2 = -alpha;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    }

    const float a0 = 1 + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    ///filter->weight = weight;
}




// Initialize second-order notch filter (struct variable, sampling frequency, center frequency, cutoff frequency)
void biquadFilterInitNotch(biquadFilter_t *filter, unsigned int samplingFreq, unsigned int filterFreq, unsigned int cutoffHz)
{
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, samplingFreq, filterFreq, Q, FILTER_NOTCH);
}





/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}

