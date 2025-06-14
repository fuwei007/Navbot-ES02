#ifndef ICM42688_H
#define ICM42688_H

#include <SPI.h>

// Define HSPI pins
#define SCK_PIN   3
#define MOSI_PIN  9
#define MISO_PIN  10
#define CS_PIN    13

// expected value in UB0_REG_WHO_AM_I reg
#define WHO_AM_I 0x47 

// Accesible from all user banks
#define REG_BANK_SEL 0x76

// User Bank 0
#define UB0_REG_DEVICE_CONFIG 0x11 
// break
#define UB0_REG_DRIVE_CONFIG 0x13 
#define UB0_REG_INT_CONFIG 0x14 
// break
#define UB0_REG_FIFO_CONFIG 0x16 
// break
#define UB0_REG_TEMP_DATA1 0x1D 
#define UB0_REG_TEMP_DATA0 0x1E 
#define UB0_REG_ACCEL_DATA_X1 0x1F 
#define UB0_REG_ACCEL_DATA_X0 0x20 
#define UB0_REG_ACCEL_DATA_Y1 0x21 
#define UB0_REG_ACCEL_DATA_Y0 0x22 
#define UB0_REG_ACCEL_DATA_Z1 0x23 
#define UB0_REG_ACCEL_DATA_Z0 0x24 
#define UB0_REG_GYRO_DATA_X1 0x25 
#define UB0_REG_GYRO_DATA_X0 0x26 
#define UB0_REG_GYRO_DATA_Y1 0x27 
#define UB0_REG_GYRO_DATA_Y0 0x28 
#define UB0_REG_GYRO_DATA_Z1 0x29 
#define UB0_REG_GYRO_DATA_Z0 0x2A 
#define UB0_REG_TMST_FSYNCH 0x2B 
#define UB0_REG_TMST_FSYNCL 0x2C 
#define UB0_REG_INT_STATUS 0x2D 
#define UB0_REG_FIFO_COUNTH 0x2E 
#define UB0_REG_FIFO_COUNTL 0x2F 
#define UB0_REG_FIFO_DATA 0x30 
#define UB0_REG_APEX_DATA0 0x31 
#define UB0_REG_APEX_DATA1 0x32 
#define UB0_REG_APEX_DATA2 0x33 
#define UB0_REG_APEX_DATA3 0x34 
#define UB0_REG_APEX_DATA4 0x35 
#define UB0_REG_APEX_DATA5 0x36 
#define UB0_REG_INT_STATUS2 0x37 
#define UB0_REG_INT_STATUS3 0x38 
// break
#define UB0_REG_SIGNAL_PATH_RESET 0x4B 
#define UB0_REG_INTF_CONFIG0 0x4C 
#define UB0_REG_INTF_CONFIG1 0x4D 
#define UB0_REG_PWR_MGMT0 0x4E 
#define UB0_REG_GYRO_CONFIG0 0x4F 
#define UB0_REG_ACCEL_CONFIG0 0x50 
#define UB0_REG_GYRO_CONFIG1 0x51 
#define UB0_REG_GYRO_ACCEL_CONFIG0 0x52 
#define UB0_REG_ACCEL_CONFIG1 0x53 
#define UB0_REG_TMST_CONFIG 0x54 
// break
#define UB0_REG_APEX_CONFIG0 0x56 
#define UB0_REG_SMD_CONFIG 0x57 
// break
#define UB0_REG_FIFO_CONFIG1 0x5F 
#define UB0_REG_FIFO_CONFIG2 0x60 
#define UB0_REG_FIFO_CONFIG3 0x61 
#define UB0_REG_FSYNC_CONFIG 0x62 
#define UB0_REG_INT_CONFIG0 0x63 
#define UB0_REG_INT_CONFIG1 0x64 
#define UB0_REG_INT_SOURCE0 0x65 
#define UB0_REG_INT_SOURCE1 0x66 
// break
#define UB0_REG_INT_SOURCE3 0x68 
#define UB0_REG_INT_SOURCE4 0x69 
// break
#define UB0_REG_FIFO_LOST_PKT0 0x6C 
#define UB0_REG_FIFO_LOST_PKT1 0x6D 
// break
#define UB0_REG_SELF_TEST_CONFIG 0x70 
// break
#define UB0_REG_WHO_AM_I 0x75 

// User Bank 1
#define UB1_REG_SENSOR_CONFIG0 0x03 
// break
#define UB1_REG_GYRO_CONFIG_STATIC2 0x0B 
#define UB1_REG_GYRO_CONFIG_STATIC3 0x0C 
#define UB1_REG_GYRO_CONFIG_STATIC4 0x0D 
#define UB1_REG_GYRO_CONFIG_STATIC5 0x0E 
#define UB1_REG_GYRO_CONFIG_STATIC6 0x0F 
#define UB1_REG_GYRO_CONFIG_STATIC7 0x10 
#define UB1_REG_GYRO_CONFIG_STATIC8 0x11 
#define UB1_REG_GYRO_CONFIG_STATIC9 0x12 
#define UB1_REG_GYRO_CONFIG_STATIC10 0x13 
// break
#define UB1_REG_XG_ST_DATA 0x5F 
#define UB1_REG_YG_ST_DATA 0x60 
#define UB1_REG_ZG_ST_DATA 0x61 
#define UB1_REG_TMSTVAL0 0x62 
#define UB1_REG_TMSTVAL1 0x63 
#define UB1_REG_TMSTVAL2 0x64 
// break
#define UB1_REG_INTF_CONFIG4 0x7A 
#define UB1_REG_INTF_CONFIG5 0x7B 
#define UB1_REG_INTF_CONFIG6 0x7C 

// User Bank 2
#define UB2_REG_ACCEL_CONFIG_STATIC2 0x03 
#define UB2_REG_ACCEL_CONFIG_STATIC3 0x04 
#define UB2_REG_ACCEL_CONFIG_STATIC4 0x05 
// break
#define UB2_REG_XA_ST_DATA 0x3B 
#define UB2_REG_YA_ST_DATA 0x3C 
#define UB2_REG_ZA_ST_DATA 0x3D 

// User Bank 4
#define UB4_REG_APEX_CONFIG1 0x40 
#define UB4_REG_APEX_CONFIG2 0x41 
#define UB4_REG_APEX_CONFIG3 0x42 
#define UB4_REG_APEX_CONFIG4 0x43 
#define UB4_REG_APEX_CONFIG5 0x44 
#define UB4_REG_APEX_CONFIG6 0x45 
#define UB4_REG_APEX_CONFIG7 0x46 
#define UB4_REG_APEX_CONFIG8 0x47 
#define UB4_REG_APEX_CONFIG9 0x48 
// break
#define UB4_REG_ACCEL_WOM_X_THR 0x4A 
#define UB4_REG_ACCEL_WOM_Y_THR 0x4B 
#define UB4_REG_ACCEL_WOM_Z_THR 0x4C 
#define UB4_REG_INT_SOURCE6 0x4D 
#define UB4_REG_INT_SOURCE7 0x4E 
#define UB4_REG_INT_SOURCE8 0x4F 
#define UB4_REG_INT_SOURCE9 0x50 
#define UB4_REG_INT_SOURCE10 0x51 
// break
#define UB4_REG_OFFSET_USER0 0x77 
#define UB4_REG_OFFSET_USER1 0x78 
#define UB4_REG_OFFSET_USER2 0x79 
#define UB4_REG_OFFSET_USER3 0x7A 
#define UB4_REG_OFFSET_USER4 0x7B 
#define UB4_REG_OFFSET_USER5 0x7C 
#define UB4_REG_OFFSET_USER6 0x7D 
#define UB4_REG_OFFSET_USER7 0x7E 
#define UB4_REG_OFFSET_USER8 0x7F 

// BANK 1
// #define GYRO_CONFIG_STATIC2 0x0B
#define GYRO_NF_ENABLE 0x00 
#define GYRO_NF_DISABLE 0x01 
#define GYRO_AAF_ENABLE 0x00 
#define GYRO_AAF_DISABLE 0x02 

// BANK 2
// #define ACCEL_CONFIG_STATIC2 0x03 
#define ACCEL_AAF_ENABLE 0x00 
#define ACCEL_AAF_DISABLE 0x01

// New configuration macro definitions
#define FSR_0             0
#define FSR_1             1
#define FSR_2             2
#define FSR_3             3
#define FSR_4             4
#define FSR_5             5
#define FSR_6             6
#define FSR_7             7

#define ODR_32KHZ         1
#define ODR_16KHZ         2
#define ODR_8KHZ          3
#define ODR_4KHZ          4
#define ODR_2KHZ          5
#define ODR_1KHZ          6
#define ODR_200HZ         7
#define ODR_100HZ         8
#define ODR_50HZ          9
#define ODR_25KHZ         10
#define ODR_12_5KHZ       11
#define ODR_6_25KHZ       12
#define ODR_3_125HZ       13
#define ODR_1_5625HZ      14
#define ODR_500HZ         15

// Structure definitions
typedef struct {
    uint8_t   accelODR: 4; 
    uint8_t   reserved: 1; 
    uint8_t   accelFsSel: 3; 
} __attribute__ ((packed)) sAccelConfig0_t;

typedef struct {
    uint8_t   gyroODR: 4; 
    uint8_t   reserved: 1; 
    uint8_t   gyroFsSel: 3; 
} __attribute__ ((packed)) sGyroConfig0_t;

typedef struct {
    uint8_t   reserved: 1; 
    uint8_t   accelDec2M2ORD: 2; 
    uint8_t   accelUIFiltORD: 2; 
    uint8_t   reserved2: 3;
} __attribute__ ((packed)) sAccelConfig1_t;

typedef struct {
    uint8_t   gyroUIFiltBW: 4; 
    uint8_t   accelUIFiltBW: 4; 
} __attribute__ ((packed)) sGyroAccelConfig0_t;

typedef struct {
    uint8_t   gyroDec2M2ODR: 2; 
    uint8_t   gyroUIFiltODR: 2; 
    uint8_t   reserved: 1; 
    uint8_t   agyroFiltBW: 3; 
} __attribute__ ((packed)) sGyroConfig1_t;

typedef struct {
    uint8_t   accelAAFDeltsqr: 4; 
    uint8_t   accelAAFBitshift: 4; 
} __attribute__ ((packed)) sAccelConfigStatic4_t;

typedef struct {
    uint8_t   accelAAFDis: 1; 
    uint8_t   accelAAFDelt: 6;
    uint8_t   reserved: 1; 
} __attribute__ ((packed)) sAccelConfigStatic2_t;

typedef struct {
    uint8_t   gyroNFDis: 1; 
    uint8_t   gyroAAFDis: 1; 
    uint8_t   reserved: 6;
} __attribute__ ((packed)) sGyroConfigStatic2_t;

typedef struct {
    uint8_t   gyroAAFDeltsqr: 4; 
    uint8_t   gyroAAFBitshift: 4; 
} __attribute__ ((packed)) sGyroConfigStatic5_t;

typedef struct {
    uint8_t   gyroNFCoswzX8: 1; 
    uint8_t   gyroNFCoswzY8: 1; 
    uint8_t   gyroNFCoswzZ8: 1;
    uint8_t   gyroNFCoswzSelX: 1;
    uint8_t   gyroNFCoswzSelY: 1;
    uint8_t   gyroNFCoswzSelZ: 1;
    uint8_t   reserved:2;
} __attribute__ ((packed)) sGyroConfigStatic9_t;

// Enumeration definitions
enum GyroFS{
    dps2000 = 0x00,
    dps1000 = 0x01,
    dps500 = 0x02,
    dps250 = 0x03,
    dps125 = 0x04,
    dps62_5 = 0x05,
    dps31_25 = 0x06,
    dps15_625 = 0x07
};

enum AccelFS{
    gpm16 = 0x00,
    gpm8 = 0x01,
    gpm4 = 0x02,
    gpm2 = 0x03
};

// Declare hspi as external variable
extern SPIClass * hspi;

// Gyroscope bias values
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;

// Function declarations
void readBytes(uint8_t reg, uint8_t *data, uint8_t length);
void writeBytes(uint8_t reg, uint8_t *data, uint8_t length);
int16_t read16BitData(uint8_t reg_high, uint8_t reg_low);
void selectUserBank(uint8_t bank);
bool initICM42688();
void readIMUData(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ, int16_t &temp);
// Gyroscope calibration function
void calibrateGyro();

#endif
