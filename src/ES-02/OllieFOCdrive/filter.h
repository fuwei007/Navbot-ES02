#ifndef __FILTER_H
#define __FILTER_H

#include <Arduino.h>

#define M_PIf       3.14159265358979323846f


typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;


float filterGetNotchQ(float centerFreq, float cutoffFreq);
//二阶滤波器初始化
void biquadFilterUpdate(biquadFilter_t *filter, unsigned int filterFreq, unsigned int refreshRate, float Q, biquadFilterType_e filterType);
//二阶陷波器初始化
void biquadFilterInitNotch(biquadFilter_t *filter, unsigned int samplingFreq, unsigned int filterFreq, unsigned int cutoffHz);
//二阶低通滤波器初始化
void biquadFilterInitLPF(biquadFilter_t *filter, unsigned int filterFreq ,unsigned int samplingFreq);
// Computes a biquad_t filter on a sample
float biquadFilterApply(biquadFilter_t *filter, float input);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, unsigned int refreshRate, float Q, biquadFilterType_e filterType);
//pt1获取滤波增益(截止频率 采样时间)
float pt1FilterGain(float f_cut, float dT);
//pt1初始化低通滤波器
void pt1FilterInit(pt1Filter_t *filter, float k);
//更新滤波增益
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
//pt1低通滤波器应用
float pt1FilterApply(pt1Filter_t *filter, float input);



#endif //__FILTER_H
