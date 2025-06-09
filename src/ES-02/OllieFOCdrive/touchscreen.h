#ifndef __touchscreen_H
#define __touchscreen_H

#include <Arduino.h>
#include "filter.h"

// 定义串口 1 的引脚
#define RXD1 20
#define TXD1 19

#define TouchDeviation 2000

typedef struct
{
  uint16_t XPressDat;
  uint16_t YPressDat; 
  uint16_t XLoosenDat;
  uint16_t YLoosenDat;
  int16_t XPdat;
  int16_t YPdat;  
  int16_t XLdat;
  int16_t YLdat;    
  float XPdatF;
  float YPdatF;
  float XPdatF_last;
  float YPdatF_last;
  uint8_t state;  
  float dt;
  uint8_t ok; 
  uint8_t rxdat[10];//接收数据的缓冲区
  uint8_t rxbuf[10];//接收数据的缓冲区
  int P_count;
  int L_count;
  int start;

} Touch_t;


void TouchscreenInit(unsigned int cutoffFreq);
void ReadTouchDat(void);
void TouchBiquadFilter(void);

extern Touch_t Touch;

#endif //
