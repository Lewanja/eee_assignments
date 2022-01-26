#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


int RXPin = 5;
int TXPin = 4;

Adafruit_MPU6050 mpu;

int GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup(void) {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  Serial1.begin(115200);
  gpsSerial.begin(GPSBaud);
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

int red_gyro(){
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
Serial.print("AcX = “); Serial.print(AcX);
Serial.print(” | AcY = “); Serial.print(AcY);
Serial.print(” | AcZ = “); Serial.print(AcZ);
Serial.print(” | Tmp = “); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
Serial.print(” | GyX = “); Serial.print(GyX);
Serial.print(” | GyY = “); Serial.print(GyY);
//Serial.print(” | GyZ = “); Serial.print(GyZ);
delay(333);
}
int red_gps(){
if (Serial1.available()) {
int inByte = Serial1.read();
Serial.write(inByte);
}

// read from port 0, send to port 1:
if (Serial.available()) {
int inByte = Serial.read();
Serial1.write(inByte);
}
}
void loop(){
red_gps();
red_gyro();
}
