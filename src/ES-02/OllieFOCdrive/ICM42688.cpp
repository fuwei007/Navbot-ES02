#include "ICM42688.h"
#include <SPI.h>
#include <Arduino.h>

// 定义 hspi 变量，用于控制 HSPI 接口进行 SPI 通信
SPIClass * hspi = new SPIClass(HSPI);

// 陀螺仪零偏值
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// 陀螺仪校准阈值，可根据实际情况调整
const int gyroCalibrationThreshold = 50; 




/**
 * @brief 从 ICM42688 传感器的指定寄存器地址开始读取多个字节的数据
 * 
 * @param reg 起始寄存器地址
 * @param data 用于存储读取数据的数组指针
 * @param length 要读取的字节数
 */
void readBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    // 拉低片选引脚，选中 ICM42688 传感器
    digitalWrite(CS_PIN, LOW);
    // 发送读命令，通过按位或操作将寄存器地址的最高位置为 1 表示读操作
    hspi->transfer(reg | 0x80); 
    // 循环读取指定长度的字节数据
    for (uint8_t i = 0; i < length; i++) {
        // 通过 SPI 总线传输 0x00 并接收传感器返回的数据
        data[i] = hspi->transfer(0x00);
    }
    // 拉高片选引脚，取消选中 ICM42688 传感器
    digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief 向 ICM42688 传感器的指定寄存器地址开始写入多个字节的数据
 * 
 * @param reg 起始寄存器地址
 * @param data 要写入的数据数组指针
 * @param length 要写入的字节数
 */
void writeBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    // 拉低片选引脚，选中 ICM42688 传感器
    digitalWrite(CS_PIN, LOW);
    // 发送要写入的寄存器地址
    hspi->transfer(reg);
    // 循环写入指定长度的字节数据
    for (uint8_t i = 0; i < length; i++) {
        // 通过 SPI 总线发送要写入的数据
        hspi->transfer(data[i]);
    }
    // 拉高片选引脚，取消选中 ICM42688 传感器
    digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief 从 ICM42688 传感器的两个连续寄存器中读取 16 位数据
 * 
 * @param reg_high 高字节寄存器地址
 * @param reg_low 低字节寄存器地址
 * @return int16_t 读取到的 16 位数据
 */
int16_t read16BitData(uint8_t reg_high, uint8_t reg_low) {
    uint8_t data[2];
    // 先读取高字节寄存器的数据
    readBytes(reg_high, data, 2);
    // 将高字节和低字节组合成 16 位数据
    return (data[0] << 8) | data[1];
}

/**
 * @brief 选择 ICM42688 传感器的用户银行
 * 
 * @param bank 要选择的用户银行编号
 */
void selectUserBank(uint8_t bank) {
    uint8_t data = bank;
    // 向 REG_BANK_SEL 寄存器写入要选择的用户银行编号
    writeBytes(REG_BANK_SEL, &data, 1);
}

/**
 * @brief 初始化 ICM42688 传感器，包括 HSPI 接口、传感器复位、设备 ID 检查、
 *        使能加速度计和陀螺仪、配置量程和采样率，以及设置低通滤波器和陷波滤波器
 * 
 * @return bool 初始化成功返回 true，失败返回 false
 */
bool initICM42688() {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    // 初始化 HSPI 接口，设置 SPI 引脚和工作模式
    hspi->begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    // 设置 SPI 数据模式为模式 3
    hspi->setDataMode(SPI_MODE3);
    // 设置 SPI 时钟分频系数，修改 SPI 时钟频率为系统时钟的 240/128 = 1.875
    hspi->setClockDivider(SPI_CLOCK_DIV32); 

    // 选择用户银行 0
    selectUserBank(0);

    // 软件复位传感器，向设备配置寄存器写入复位命令
    uint8_t resetData = 0x01;
    writeBytes(UB0_REG_DEVICE_CONFIG, &resetData, 1);
    // 延时 100 毫秒，等待复位操作完成
    delay(100);

    // 检查设备 ID，从 WHO_AM_I 寄存器读取设备 ID
    uint8_t whoAmI;
    readBytes(UB0_REG_WHO_AM_I, &whoAmI, 1);
    // 比较读取到的设备 ID 是否与预设值一致
    if (whoAmI != WHO_AM_I) {
        // 不一致则打印错误信息并返回 false
        Serial.println("ICM42688 device ID check failed!");
        return false;
    }

    // 使能加速度计和陀螺仪，向电源管理寄存器写入使能命令
    uint8_t pwrData = 0x0F;
    writeBytes(UB0_REG_PWR_MGMT0, &pwrData, 1);
    // 延时 100 毫秒，等待使能操作生效
    delay(100);

    // 配置加速度计
    // 设置加速度计量程为±8g，采样率为 1KHZ
    sAccelConfig0_t accelConfig0;
    accelConfig0.accelODR = ODR_1KHZ;
    accelConfig0.accelFsSel = gpm8;
    accelConfig0.reserved = 0;
    // 将结构体数据转换为字节数据
    uint8_t accelConfigByte = *((uint8_t*)&accelConfig0);
    // 向加速度计配置寄存器 0 写入配置数据
    writeBytes(UB0_REG_ACCEL_CONFIG0, &accelConfigByte, 1);

    // 配置陀螺仪
    // 设置陀螺仪量程为±2000dps，采样率为 1KHZ
    sGyroConfig0_t gyroConfig0;
    gyroConfig0.gyroODR = ODR_1KHZ;
    gyroConfig0.gyroFsSel = dps2000;
    gyroConfig0.reserved = 0;
    // 将结构体数据转换为字节数据
    uint8_t gyroConfigByte = *((uint8_t*)&gyroConfig0);
    // 向陀螺仪配置寄存器 0 写入配置数据
    writeBytes(UB0_REG_GYRO_CONFIG0, &gyroConfigByte, 1);

    // 配置加速度计和陀螺仪的低通滤波器
    // 加速度计 LPF 带宽为 max(400Hz, ODR)/4
    uint8_t accelLPF = 1;
    // 陀螺仪 LPF 带宽为 max(400Hz, ODR)/4
    uint8_t gyroLPF = 1;
    sGyroAccelConfig0_t gyroAccelConfig0;
    gyroAccelConfig0.accelUIFiltBW = accelLPF;
    gyroAccelConfig0.gyroUIFiltBW = gyroLPF;
    // 将结构体数据转换为字节数据
    uint8_t gyroAccelConfigByte = *((uint8_t*)&gyroAccelConfig0);
    // 向陀螺仪和加速度计配置寄存器 0 写入配置数据
    writeBytes(UB0_REG_GYRO_ACCEL_CONFIG0, &gyroAccelConfigByte, 1);

    // 配置陀螺仪陷波滤波器
    // 陀螺仪陷波滤波器启用，抗混叠滤波器启用
    uint8_t gyroNotchConfig = 0x00;
    // 选择用户银行 1，因为陷波滤波器配置寄存器在用户银行 1 中
    selectUserBank(1);
    sGyroConfigStatic2_t gyroConfigStatic2;
    // 通过位操作提取陷波滤波器使能标志
    gyroConfigStatic2.gyroNFDis = (gyroNotchConfig & 0x01);
    // 通过位操作提取抗混叠滤波器使能标志
    gyroConfigStatic2.gyroAAFDis = (gyroNotchConfig & 0x02) >> 1;
    gyroConfigStatic2.reserved = 0;
    // 将结构体数据转换为字节数据
    uint8_t gyroConfigStatic2Byte = *((uint8_t*)&gyroConfigStatic2);
    // 向陀螺仪静态配置寄存器 2 写入配置数据
    writeBytes(UB1_REG_GYRO_CONFIG_STATIC2, &gyroConfigStatic2Byte, 1);

    // 回到用户银行 0
    selectUserBank(0);

    return true;
}

/**
 * @brief 一次性读取 ICM42688 传感器的六轴数据（加速度计三轴和陀螺仪三轴）以及温度数据
 * 
 * @param accelX 用于存储加速度计 X 轴数据的引用
 * @param accelY 用于存储加速度计 Y 轴数据的引用
 * @param accelZ 用于存储加速度计 Z 轴数据的引用
 * @param gyroX 用于存储陀螺仪 X 轴数据的引用
 * @param gyroY 用于存储陀螺仪 Y 轴数据的引用
 * @param gyroZ 用于存储陀螺仪 Z 轴数据的引用
 * @param temp 用于存储温度数据的引用
 */
void readIMUData(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ, int16_t &temp) {
    // 选择用户银行 0
    selectUserBank(0);
    uint8_t data[14];
    // 从温度数据寄存器开始读取 14 个字节的数据
    readBytes(UB0_REG_TEMP_DATA1, data, 14);

    // 解析温度数据
    temp = (data[0] << 8) | data[1];
    // 解析加速度计 X 轴数据
    accelX = (data[2] << 8) | data[3];
    // 解析加速度计 Y 轴数据
    accelY = (data[4] << 8) | data[5];
    // 解析加速度计 Z 轴数据
    accelZ = (data[6] << 8) | data[7];
    // 解析陀螺仪 X 轴数据
    gyroX = (data[8] << 8) | data[9];
    // 解析陀螺仪 Y 轴数据
    gyroY = (data[10] << 8) | data[11];
    // 解析陀螺仪 Z 轴数据
    gyroZ = (data[12] << 8) | data[13];
}





// 陀螺仪校准函数
void calibrateGyro() {
    Serial.println("Calibrating gyroscope. Keep the device still...");
    const int calibrationSamples = 1000;  // 采集的样本数量
    int16_t gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
    int validSamples = 0;

    while (validSamples < calibrationSamples) {
        int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp;
        readIMUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);

        // 检查是否有较大的变化，判断设备是否移动
        if (abs(gyroX) < gyroCalibrationThreshold && 
            abs(gyroY) < gyroCalibrationThreshold && 
            abs(gyroZ) < gyroCalibrationThreshold) {
            gyroX_sum += gyroX;
            gyroY_sum += gyroY;
            gyroZ_sum += gyroZ;
            validSamples++;
        } else {
            // 设备移动，重新开始计数
            Serial.print(" | x: ");
            Serial.print(gyroX);
            Serial.print(" y: ");
            Serial.print(gyroY);
            Serial.print(" Z: ");
            Serial.print(gyroZ);            
            Serial.println("Device moved during calibration. Restarting...");
            gyroX_sum = 0;
            gyroY_sum = 0;
            gyroZ_sum = 0;
            validSamples = 0;
            
        }

        delay(3);  // 等待一段时间再采集下一个样本
    }

    // 计算零偏值
    gyroBiasX = (float)gyroX_sum / calibrationSamples;
    gyroBiasY = (float)gyroY_sum / calibrationSamples;
    gyroBiasZ = (float)gyroZ_sum / calibrationSamples;

    Serial.println("Gyroscope calibration completed.");
}
