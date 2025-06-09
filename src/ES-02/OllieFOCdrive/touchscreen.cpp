#include "touchscreen.h"

Touch_t Touch;
biquadFilter_t TouchLPF[2];//二阶低通滤波器


uint8_t TouchOk = 0;
uint8_t TouchList = 0;

void TouchscreenInit(unsigned int cutoffFreq)
{
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    biquadFilterInitLPF(&TouchLPF[0], (unsigned int)100, (unsigned int)cutoffFreq);
    biquadFilterInitLPF(&TouchLPF[1], (unsigned int)100, (unsigned int)cutoffFreq);
}


void ReadTouchDat(void) //读触屏数据数据
{  
  /*
  // 用于记录已读取的字节数
  int bytesRead = 0;
  // 检查串口缓冲区中是否有可用数据
  if (Serial1.available() >= 5) 
  {
    // 等待直到读取到5个字节的数据
    while (bytesRead < 5) 
    {
       // 读取一个字节的数据
      Touch.rxdat[bytesRead] = Serial1.read();
      // 已读取字节数加1
      bytesRead++; 
    }
 
  }

  */
  uint8_t dat;
  if(Serial1.available() > 0)
  {
    dat = Serial1.read();
    if((dat == 0x81)||(dat == 0x80))
    {
        TouchOk = 1;
        TouchList = 0;
        Touch.rxbuf[TouchList] = dat;
        Touch.rxbuf[1] = 0;
        Touch.rxbuf[2] = 0;
        Touch.rxbuf[3] = 0;
        Touch.rxbuf[4] = 0;
        
    }
    else if(TouchOk == 1)//按下数据
    {
        TouchList++;
        Touch.rxbuf[TouchList] = dat;
  
        if(TouchList >=4)
        if(Touch.ok==0)
        {
          
          Touch.YPressDat = Touch.rxbuf[1];
          Touch.YPressDat<<=8;
          Touch.YPressDat+=Touch.rxbuf[2];
          
          Touch.XPressDat = Touch.rxbuf[3];
          Touch.XPressDat<<=8;
          Touch.XPressDat+=Touch.rxbuf[4];        
          
          if(1)//((Touch.XPressDat!=0)&&(Touch.YPressDat!=0))
          {
            if(Touch.rxbuf[0]==0x81)
            {
              
                Touch.state = 1;
                Touch.XPdat = Touch.XPressDat-TouchDeviation;
                Touch.YPdat = Touch.YPressDat-TouchDeviation;
                
                /*
                Serial.print("  aX:");
                Serial.print(Touch.XPdat); 
                Serial.print("  aY:");
                Serial.println(Touch.YPdat);  
                */
            }
            else if(Touch.rxbuf[0]==0x80)
            {
                Touch.state = 0;
                Touch.XLdat = Touch.XPressDat-TouchDeviation;
                Touch.YLdat = Touch.YPressDat-TouchDeviation;
                /*
                Serial.print("  tX:");
                Serial.print(Touch.XLdat); 
                Serial.print("  tX:");
                Serial.println(Touch.YLdat);      
                */        
            }       
          }
  
  
              Touch.ok = 0; 
              TouchList = 0;
          TouchOk = 0;
          
        }
  
    }
    else 
    {
        TouchList = 0;
        TouchOk = 0; 
        Touch.rxbuf[0] = 0;
        Touch.rxbuf[1] = 0;
        Touch.rxbuf[2] = 0;
        Touch.rxbuf[3] = 0;
        Touch.rxbuf[4] = 0;   
    }
      
  }    

}

void TouchBiquadFilter(void)
{
  Touch.XPdatF = biquadFilterApply(&TouchLPF[0], Touch.XPdat);
  Touch.YPdatF = biquadFilterApply(&TouchLPF[1], Touch.YPdat); 
}
