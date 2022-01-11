#define delayInMS (100)
#define baudRate (115200)

void setup() {
  Serial.begin(baudRate);
  IMU.begin(); // turns IMU sensor on
  initFingers();
  rotationType = QuaternionMode;
  prevElapsedTime = millis();
  delay(delayInMS);
  processing = true;
}

void loop() {
  // data needed for calibrating the sensor.
  uint8_t system, gyro, accel, mg = 0;
  IMU.getCalibration(&system, &gyro, &accel, &mg);
  
  if (rotationType == QuaternionMode)
  {
    getFlexSensorData();
    quaternionData = IMU.getQuat();
    setImuQuaternionMatrix(quaternionData, system, gyro, accel, mg);

    printQuaternionMatrix();
  }
  else if (rotationType == EulerMode)
  {
    rawAccelerometerData = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    rawGyroscopeData = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    rawMagnetometerData = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    setRawAccelGyroMagMatrix(rawAccelerometerData, rawGyroscopeData, rawMagnetometerData);

    // TESTING to eventually get to the roll pitch yaw function working nicely hopefully.
    // printRawAccelGyroMagMatrix();
    // rawPrintAccelerationData();
    // calculateAndPrintRollPitch();
    // calculateAndPrintRollPitchWithFilter();
    // calculateAndPrintRollPitchWithGyroscope();
    // calculateAndPrintRollPitchWithAcceleromterGyroscopeFilter();
    calculateRollPitchYaw(); // Yaw too shakey with this method for my liking, but keeping it for now and will adjust if time. Pitch Yaw work decent.
    printCalibrationData(system, gyro, accel, mg);
    Serial.println();
  }
  
  delay(delayInMS);
}
