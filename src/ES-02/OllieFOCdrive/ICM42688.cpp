#include "ICM42688.h"
#include <SPI.h>
#include <Arduino.h>

// Define hspi variable for controlling HSPI interface for SPI communication
SPIClass * hspi = new SPIClass(HSPI);

// Gyroscope bias values
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// Gyroscope calibration threshold, can be adjusted based on actual conditions
const int gyroCalibrationThreshold = 50; 




/**
 * @brief Read multiple bytes of data starting from the specified register address of the ICM42688 sensor
 * 
 * @param reg Starting register address
 * @param data Pointer to array for storing read data
 * @param length Number of bytes to read
 */
void readBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    // Pull CS pin low to select ICM42688 sensor
    digitalWrite(CS_PIN, LOW);
    // Send read command by setting the highest bit of register address to 1 through bitwise OR operation
    hspi->transfer(reg | 0x80); 
    // Loop to read specified number of bytes
    for (uint8_t i = 0; i < length; i++) {
        // Transfer 0x00 through SPI bus and receive data returned by sensor
        data[i] = hspi->transfer(0x00);
    }
    // Pull CS pin high to deselect ICM42688 sensor
    digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief Write multiple bytes of data starting from the specified register address of the ICM42688 sensor
 * 
 * @param reg Starting register address
 * @param data Pointer to array containing data to write
 * @param length Number of bytes to write
 */
void writeBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    // Pull CS pin low to select ICM42688 sensor
    digitalWrite(CS_PIN, LOW);
    // Send register address to write
    hspi->transfer(reg);
    // Loop to write specified number of bytes
    for (uint8_t i = 0; i < length; i++) {
        // Send data to write through SPI bus
        hspi->transfer(data[i]);
    }
    // Pull CS pin high to deselect ICM42688 sensor
    digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief Read 16-bit data from two consecutive registers of the ICM42688 sensor
 * 
 * @param reg_high High byte register address
 * @param reg_low Low byte register address
 * @return int16_t 16-bit data read
 */
int16_t read16BitData(uint8_t reg_high, uint8_t reg_low) {
    uint8_t data[2];
    // First read data from high byte register
    readBytes(reg_high, data, 2);
    // Combine high byte and low byte into 16-bit data
    return (data[0] << 8) | data[1];
}

/**
 * @brief Select user bank of ICM42688 sensor
 * 
 * @param bank User bank number to select
 */
void selectUserBank(uint8_t bank) {
    uint8_t data = bank;
    // Write user bank number to REG_BANK_SEL register
    writeBytes(REG_BANK_SEL, &data, 1);
}

/**
 * @brief Initialize ICM42688 sensor, including HSPI interface, sensor reset, device ID check,
 *        enable accelerometer and gyroscope, configure range and sampling rate, and set up
 *        low-pass filter and notch filter
 * 
 * @return bool Returns true if initialization successful, false if failed
 */
bool initICM42688() {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    // Initialize HSPI interface, set SPI pins and working mode
    hspi->begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    // Set SPI data mode to mode 3
    hspi->setDataMode(SPI_MODE3);
    // Set SPI clock divider, modify SPI clock frequency to system clock/32
    hspi->setClockDivider(SPI_CLOCK_DIV32); 

    // Select user bank 0
    selectUserBank(0);

    // Software reset sensor, write reset command to device configuration register
    uint8_t resetData = 0x01;
    writeBytes(UB0_REG_DEVICE_CONFIG, &resetData, 1);
    // Delay 100ms, wait for reset operation to complete
    delay(100);

    // Check device ID, read device ID from WHO_AM_I register
    uint8_t whoAmI;
    readBytes(UB0_REG_WHO_AM_I, &whoAmI, 1);
    // Compare if read device ID matches preset value
    if (whoAmI != WHO_AM_I) {
        // If not matching, print error message and return false
        Serial.println("ICM42688 device ID check failed!");
        return false;
    }

    // Enable accelerometer and gyroscope, write enable command to power management register
    uint8_t pwrData = 0x0F;
    writeBytes(UB0_REG_PWR_MGMT0, &pwrData, 1);
    // Delay 100ms, wait for enable operation to take effect
    delay(100);

    // Configure accelerometer
    // Set accelerometer range to ±8g, sampling rate to 1KHZ
    sAccelConfig0_t accelConfig0;
    accelConfig0.accelODR = ODR_1KHZ;
    accelConfig0.accelFsSel = gpm8;
    accelConfig0.reserved = 0;
    // Convert structure data to byte data
    uint8_t accelConfigByte = *((uint8_t*)&accelConfig0);
    // Write configuration data to accelerometer configuration register 0
    writeBytes(UB0_REG_ACCEL_CONFIG0, &accelConfigByte, 1);

    // Configure gyroscope
    // Set gyroscope range to ±2000dps, sampling rate to 1KHZ
    sGyroConfig0_t gyroConfig0;
    gyroConfig0.gyroODR = ODR_1KHZ;
    gyroConfig0.gyroFsSel = dps2000;
    gyroConfig0.reserved = 0;
    // Convert structure data to byte data
    uint8_t gyroConfigByte = *((uint8_t*)&gyroConfig0);
    // Write configuration data to gyroscope configuration register 0
    writeBytes(UB0_REG_GYRO_CONFIG0, &gyroConfigByte, 1);

    // Configure low-pass filter for accelerometer and gyroscope
    // Accelerometer LPF bandwidth is max(400Hz, ODR)/4
    uint8_t accelLPF = 1;
    // Gyroscope LPF bandwidth is max(400Hz, ODR)/4
    uint8_t gyroLPF = 1;
    sGyroAccelConfig0_t gyroAccelConfig0;
    gyroAccelConfig0.accelUIFiltBW = accelLPF;
    gyroAccelConfig0.gyroUIFiltBW = gyroLPF;
    // Convert structure data to byte data
    uint8_t gyroAccelConfigByte = *((uint8_t*)&gyroAccelConfig0);
    // Write configuration data to gyroscope and accelerometer configuration register 0
    writeBytes(UB0_REG_GYRO_ACCEL_CONFIG0, &gyroAccelConfigByte, 1);

    // Configure gyroscope notch filter
    // Enable gyroscope notch filter, enable anti-aliasing filter
    uint8_t gyroNotchConfig = 0x00;
    // Select user bank 1, as notch filter configuration register is in user bank 1
    selectUserBank(1);
    sGyroConfigStatic2_t gyroConfigStatic2;
    // Extract notch filter enable flag through bit operation
    gyroConfigStatic2.gyroNFDis = (gyroNotchConfig & 0x01);
    // Extract anti-aliasing filter enable flag through bit operation
    gyroConfigStatic2.gyroAAFDis = (gyroNotchConfig & 0x02) >> 1;
    gyroConfigStatic2.reserved = 0;
    // Convert structure data to byte data
    uint8_t gyroConfigStatic2Byte = *((uint8_t*)&gyroConfigStatic2);
    // Write configuration data to gyroscope static configuration register 2
    writeBytes(UB1_REG_GYRO_CONFIG_STATIC2, &gyroConfigStatic2Byte, 1);

    // Return to user bank 0
    selectUserBank(0);

    return true;
}

/**
 * @brief Read six-axis data (accelerometer three-axis and gyroscope three-axis) and temperature data from ICM42688 sensor at once
 * 
 * @param accelX Reference for storing accelerometer X-axis data
 * @param accelY Reference for storing accelerometer Y-axis data
 * @param accelZ Reference for storing accelerometer Z-axis data
 * @param gyroX Reference for storing gyroscope X-axis data
 * @param gyroY Reference for storing gyroscope Y-axis data
 * @param gyroZ Reference for storing gyroscope Z-axis data
 * @param temp Reference for storing temperature data
 */
void readIMUData(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ, int16_t &temp) {
    // Select user bank 0
    selectUserBank(0);
    uint8_t data[14];
    // Read 14 bytes of data starting from temperature data register
    readBytes(UB0_REG_TEMP_DATA1, data, 14);

    // Parse temperature data
    temp = (data[0] << 8) | data[1];
    // Parse accelerometer X-axis data
    accelX = (data[2] << 8) | data[3];
    // Parse accelerometer Y-axis data
    accelY = (data[4] << 8) | data[5];
    // Parse accelerometer Z-axis data
    accelZ = (data[6] << 8) | data[7];
    // Parse gyroscope X-axis data
    gyroX = (data[8] << 8) | data[9];
    // Parse gyroscope Y-axis data
    gyroY = (data[10] << 8) | data[11];
    // Parse gyroscope Z-axis data
    gyroZ = (data[12] << 8) | data[13];
}





// Gyroscope calibration function
void calibrateGyro() {
    Serial.println("Calibrating gyroscope. Keep the device still...");
    const int calibrationSamples = 1000;  // Number of samples to collect
    int16_t gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
    int validSamples = 0;

    while (validSamples < calibrationSamples) {
        int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp;
        readIMUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);

        // Check for large changes to determine if device is moving
        if (abs(gyroX) < gyroCalibrationThreshold && 
            abs(gyroY) < gyroCalibrationThreshold && 
            abs(gyroZ) < gyroCalibrationThreshold) {
            gyroX_sum += gyroX;
            gyroY_sum += gyroY;
            gyroZ_sum += gyroZ;
            validSamples++;
        } else {
            // Device moved, restart counting
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

        delay(3);  // Wait for a while before collecting next sample
    }

    // Calculate bias values
    gyroBiasX = (float)gyroX_sum / calibrationSamples;
    gyroBiasY = (float)gyroY_sum / calibrationSamples;
    gyroBiasZ = (float)gyroZ_sum / calibrationSamples;

    Serial.println("Gyroscope calibration completed.");
}
