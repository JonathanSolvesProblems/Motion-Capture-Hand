#define normalize (9.8) // convert from m/s^2 to g
#define radiansToDegrees ((1)/(PI)*(180))
#define DegreesToRadians ((PI)/(180))
#define fingerThreshold (250)

void setImuQuaternionMatrix(imu::Quaternion quaternionData, uint8_t systemInput, uint8_t gyroInput, uint8_t accelInput, uint8_t magInput)
{
  if (processing && rotationType == QuaternionMode)
  {
    imuQuaternionMatrix[0][0] = quaternionData.w();
    imuQuaternionMatrix[0][1] = quaternionData.x();
    imuQuaternionMatrix[0][2] = quaternionData.y();
    imuQuaternionMatrix[0][3] = quaternionData.z();
    // calibration info
    imuQuaternionMatrix[1][0] = systemInput;
    imuQuaternionMatrix[1][1] = gyroInput;
    imuQuaternionMatrix[1][2] = accelInput;
    imuQuaternionMatrix[1][3] = magInput;
  }
}

void printQuaternionMatrix()
{
  for(uint8_t height = 0; height < quaternionMatrixHeight; height++)
  {
    for(uint8_t width = 0; width < quaternionMatrixWeight; width++)
    {
      Serial.print(imuQuaternionMatrix[height][width]);
      Serial.print(",");
    }
    if(height == (quaternionMatrixHeight - 1))
    {
      Serial.println();
    }
  }
}

void setRawAccelGyroMagMatrix(imu::Vector<axisNum> rawAccelerometerData, imu::Vector<axisNum> rawGyroscopeData, imu::Vector<axisNum> rawMagnetometerData)
{
  // raw accelerometer data
  rawAccelGyroMagData[0][0] = rawAccelerometerData.x();
  rawAccelGyroMagData[0][1] = rawAccelerometerData.y();
  rawAccelGyroMagData[0][2] = rawAccelerometerData.z();

  // raw gyroscope data
  rawAccelGyroMagData[1][0] = rawGyroscopeData.x();
  rawAccelGyroMagData[1][1] = rawGyroscopeData.y();
  rawAccelGyroMagData[1][2] = rawGyroscopeData.z();

  // raw magnetimeter data
  rawAccelGyroMagData[2][0] = rawMagnetometerData.x();
  rawAccelGyroMagData[2][1] = rawMagnetometerData.y();
  rawAccelGyroMagData[2][2] = rawMagnetometerData.z();
}

void printRawAccelGyroMagMatrix()
{
  for(uint8_t height = 0; height < axisNum; height++)
  {
    for(uint8_t width = 0; width < axisNum; width++)
    {
      Serial.print(rawAccelGyroMagData[height][width]);
      Serial.print(",");
    }
    if(height == (axisNum - 1))
    {
      Serial.println();
    }
  }
}

void initFingers()
{
  pinkyFinger = A0;
  ringFinger = A1;
  middleFinger = A2;
  indexFinger = A3;
  thumbFinger = A7;
}

void getPinkyFingerData()
{
  uint16_t mapRange[4] = {350, 482, 90, 0};
  
  uint16_t pinkyFingerFlex = analogRead(pinkyFinger);
  // Serial.println(fingerFlex);
  uint8_t flexAngle = map(pinkyFingerFlex, mapRange[0], mapRange[1], mapRange[2], mapRange[3]);

  if (flexAngle >= fingerThreshold)
     flexAngle = 0;

  Serial.print(flexAngle);
  Serial.print(",");
  
}

void getRingFingerData()
{
  uint16_t mapRange[4] = {230, 460, 90, 0};
  
  uint16_t ringFingerFlex = analogRead(ringFinger);
  uint8_t flexAngle = map(ringFingerFlex, mapRange[0], mapRange[1], mapRange[2], mapRange[3]);

  if (flexAngle >= fingerThreshold)
     flexAngle = 0;

  Serial.print(flexAngle);
  Serial.print(",");
  
}

void getMiddleFingerData()
{
  uint16_t mapRange[4] = {280, 482, 90, 0};
  
  uint16_t middleFingerFlex = analogRead(middleFinger);
  uint8_t flexAngle = map(middleFingerFlex, mapRange[0], mapRange[1], mapRange[2], mapRange[3]);

  if (flexAngle >= fingerThreshold)
     flexAngle = 0;

  Serial.print(flexAngle);
  Serial.print(",");
  
}

void getIndexFingerData()
{
  uint16_t mapRange[4] = {250, 480, 90, 0};
  
  uint16_t indexFingerFlex = analogRead(indexFinger);
  uint8_t flexAngle = map(indexFingerFlex, mapRange[0], mapRange[1], mapRange[2], mapRange[3]);

  if (flexAngle >= fingerThreshold)
     flexAngle = 0;

  Serial.print(flexAngle);
  Serial.print(",");
  
}

void getThumbFingerData()
{
  uint16_t mapRange[4] = {280, 488, 90, 0};
  
  uint16_t thumbFingerFlex = analogRead(thumbFinger);
  uint8_t flexAngle = map(thumbFingerFlex, mapRange[0], mapRange[1], mapRange[2], mapRange[3]);

  if (flexAngle >= fingerThreshold)
     flexAngle = 0;

  Serial.print(flexAngle);
  Serial.print(",");
  
}

void getFlexSensorData()
{
  getPinkyFingerData();
  getRingFingerData();
  getMiddleFingerData();
  getIndexFingerData();
  getThumbFingerData();
}

void rawPrintAccelerationData()
{
  uint8_t height = 0;
  
  for(uint8_t width = 0; width < axisNum; width++)
  {
    Serial.print(rawAccelGyroMagData[height][width]);
    Serial.print(",");
  }
}

void printCalibrationData(uint8_t systemInput, uint8_t gyroInput, uint8_t accelInput, uint8_t magInput)
{
  calibrationData[0] = systemInput;
  calibrationData[1] = gyroInput;
  calibrationData[2] = accelInput;
  calibrationData[3] = magInput;

  for(uint8_t index = 0; index < calibrationDataSize; index++)
  {
    Serial.print(calibrationData[index]);
    Serial.print(",");
  }
}

float roll;
float pitch;

// Compute the roll and pitch of the raw accelerometer data
void calculateAndPrintRollPitch()
{
  roll = -atan2(rawAccelGyroMagData[0][0] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;
  pitch = atan2(rawAccelGyroMagData[0][1] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
}

float currentRoll, currentPitch, filteredRoll, filteredPitch;
float rollCorrection, pitchCorrection = 0;
float lowFilter = 0.85, highFilter = 0.15;

// Filter to reduce noise of the accelerometer data.
void calculateAndPrintRollPitchWithFilter()
{
  lowFilter = 0.85, highFilter = 0.15;
 
  currentRoll = -atan2(rawAccelGyroMagData[0][0] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;
  currentPitch = atan2(rawAccelGyroMagData[0][1] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;

  filteredRoll = (currentRoll * highFilter) + (rollCorrection * lowFilter);
  filteredPitch = (currentPitch * highFilter) + (pitchCorrection * lowFilter);

  Serial.print(filteredRoll);
  Serial.print(",");
  Serial.print(filteredPitch);
  Serial.print(",");

  rollCorrection = filteredRoll;
  pitchCorrection = filteredPitch;
}

float rollGyro, pitchGyro = 0;

// calculate roll and pitch with gyroscope
void calculateAndPrintRollPitchWithGyroscope()
{
  elapsedTime = (millis() - prevElapsedTime) * MSToSeconds;
  prevElapsedTime = millis();

  rollGyro += rawAccelGyroMagData[1][1] * elapsedTime;
  pitchGyro -= rawAccelGyroMagData[1][0] * elapsedTime;

  Serial.print(rollGyro);
  Serial.print(",");
  Serial.print(pitchGyro);
  Serial.print(",");
}

float rollBoth, pitchBoth = 0;
float lowFilterBoth = 0.95, highFilterBoth = 0.05;
// calculate roll and pitch with both the acceleromter and gyroscope data filtered.
void calculateAndPrintRollPitchWithAcceleromterGyroscopeFilter()
{
  currentRoll = -atan2(rawAccelGyroMagData[0][0] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;
  currentPitch = atan2(rawAccelGyroMagData[0][1] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;

  filteredRoll = (currentRoll * highFilter) + (rollCorrection * lowFilter);
  filteredPitch = (currentPitch * highFilter) + (pitchCorrection * lowFilter);
  
  elapsedTime = (millis() - prevElapsedTime) * MSToSeconds;
  prevElapsedTime = millis();

  rollBoth = (rollBoth + rawAccelGyroMagData[1][1] * elapsedTime) * lowFilterBoth + (currentRoll * highFilterBoth);
  pitchBoth = (pitchBoth - rawAccelGyroMagData[1][0] * elapsedTime) * lowFilterBoth + (currentPitch * highFilterBoth);

  Serial.print(rollBoth);
  Serial.print(",");
  Serial.print(pitchBoth);
  Serial.print(",");
}

float yawX, yawY, yaw = 0;
float rollInRadians, pitchInRadians = 0;

float calculateRollPitchYaw()
{
  currentRoll = -atan2(rawAccelGyroMagData[0][0] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;
  currentPitch = atan2(rawAccelGyroMagData[0][1] / normalize, rawAccelGyroMagData[0][2] / normalize) * radiansToDegrees;
  
  elapsedTime = (millis() - prevElapsedTime) * MSToSeconds;
  prevElapsedTime = millis();

  rollBoth = (rollBoth + rawAccelGyroMagData[1][1] * elapsedTime) * lowFilterBoth + (currentRoll * highFilterBoth);
  pitchBoth = (pitchBoth - rawAccelGyroMagData[1][0] * elapsedTime) * lowFilterBoth + (currentPitch * highFilterBoth);

  rollInRadians = rollBoth * DegreesToRadians;
  pitchInRadians = pitchBoth * DegreesToRadians;
 
  yawX = rawAccelGyroMagData[2][0] * cos(rollInRadians) - rawAccelGyroMagData[2][1] * sin(pitchInRadians) * sin(rollInRadians) + rawAccelGyroMagData[2][2] * cos(pitchInRadians) * sin(rollInRadians);
  yawY = rawAccelGyroMagData[2][1] * cos(pitchInRadians) + rawAccelGyroMagData[2][2] * sin(pitchInRadians);

  yaw = atan2(yawY, yawX) * radiansToDegrees;
  
  Serial.print(rollBoth);
  Serial.print(",");
  Serial.print(pitchBoth);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");
}
