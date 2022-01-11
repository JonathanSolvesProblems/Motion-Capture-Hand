#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define IMU_DelayInMs (100)
#define quaternionMatrixHeight (2)
#define quaternionMatrixWeight (4)
#define calibrationDataSize (4)
#define axisNum (3)
#define MSToSeconds ((1)/(1000.0))

Adafruit_BNO055 IMU = Adafruit_BNO055();

imu::Vector<axisNum> rawAccelerometerData;
imu::Vector<axisNum> rawGyroscopeData;
imu::Vector<axisNum> rawMagnetometerData;
imu::Quaternion quaternionData;

float imuQuaternionMatrix[quaternionMatrixHeight][quaternionMatrixWeight];
float rawAccelGyroMagData[axisNum][axisNum];
float calibrationData[calibrationDataSize];
float flexSensorData;

// for euler implementation, mainly gyroscope
float elapsedTime;
uint16_t prevElapsedTime;

// extra user protection to prevent methods from being called before setup.
bool processing = false; 

enum RotationType {
    QuaternionMode = 1,
    EulerMode = 2
};

RotationType rotationType;

uint16_t pinkyFinger;
uint16_t ringFinger;
uint16_t middleFinger;
uint16_t indexFinger;
uint16_t thumbFinger;

void setImuQuaternionMatrix(imu::Quaternion quaternionData, uint8_t systemInput, uint8_t gyroInput, uint8_t accelInput, uint8_t magInput);
void printQuaternionMatrix();
void setRawAccelGyroMagMatrix(imu::Vector<axisNum> rawAccelerometerData, imu::Vector<axisNum> rawGyroscopeData, imu::Vector<axisNum> rawMagnetometerData);
void initFingers();
void getFlexSensorData();
void rawPrintAccelerationData();
void printCalibrationData(uint8_t systemInput, uint8_t gyroInput, uint8_t accelInput, uint8_t magInput);
void calculateAndPrintRollPitch();
void calculateAndPrintRollPitchWithFilter();
void calculateAndPrintRollPitchWithGyroscope();
void calculateAndPrintRollPitchWithAcceleromterGyroscopeFilter();
float calculateRollPitchYaw();
