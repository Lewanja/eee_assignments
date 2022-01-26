#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include "MPU9250.h"
#include <cmath>
#include <stdlib.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <iostream>
//#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PulseSensorPlayground.h>  // Includes the PulseSensorPlayground Library
#define USE_ARDUINO_INTERRUPTS true // Set-up low-level interrupts for most acurate BPM math

const int PulseWire = 0; // 'S' Signal pin connected to A0
const int LED13 = 13;    // The on-board Arduino LED
int Threshold = 550;     // Determine which Signal to "count as a beat" and which to ignore

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

float temperature;

#define ss 5
#define rst 14
#define dio0 2

// GPS DATA STRUCT
struct GPS_DATA
{
  short status;
  float lat;
  float lon;
};

typedef struct GPS_DATA GPS;

// Helper functions declaration
int read_ACCELL_GYRO(void);
struct GPS_DATA read_GPS();

//HardwarSerial UART GPIO
int RXPin = 16;
int TXPin = 17;

// GPS Software Serial baud rate
int gpsBaud = 9600;
/**
 * Sensor Callibration
 * Use zero offsets to adjust zero errors
*/
float accel_x_offset = +17.43;
float accel_y_offset = +9.50;
float accel_z_offset = 18.51;

/**Radian to degree conversion factor*/
float rad2Deg = 57.2958;

/**
 * Acceleration and rotational speed threshold values
 * Adjust the values to change sensitivity
*/
float ACCEL_FT = 29;
float ROT_FT = 60;

/** Initialize MPU sensor object on I2C Bus address 0x68
 * Pull AD0 high to use Bus Address 0x69
*/
MPU9250 IMU(Wire, 0x68);
// Create a TinyGPS++ object
TinyGPSPlus gps;
PulseSensorPlayground pulseSensor; // Creates an object

void setup()
{
  Serial.begin(115200);
  // Create a hardware serial port called "Serial1"
  Serial1.begin(gpsBaud, SERIAL_8N1, RXPin, TXPin);
  // Configure the PulseSensor object, by assigning our variables to it
  pulseSensor.analogInput(PulseWire); // Blink on-board LED with heartbeat
  pulseSensor.setThreshold(Threshold);

  /** Initialize bme*/
  while (!bme.begin(0x76))
  {
    yield();
  }
  Serial.println("BME_OK");

  /** Initialize Communication with MPU*/
  while (!IMU.begin())
  {
    yield();
  }
  Serial.println("MPU_OK");

  LoRa.setPins(ss, rst, dio0); //setup LoRa transceiver module

  while (!LoRa.begin(868100000)) //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0x39);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125000);

  Serial.println("RMF INIT_OK!");
}

void loop()
{

  String data_str = String(""); // data container

  float gps_lat = 0;
  float gps_lon = 0;
  float gps_status = 0;

  int FALL_STATUS = read_ACCELL_GYRO(); // read mpu and fall status

  if (FALL_STATUS)
  {
    Serial.print("FALL: ");
    Serial.println(FALL_STATUS);

    float temp = bme.readTemperature();
    int myBPM = pulseSensor.getBeatsPerMinute();

    while (Serial1.available() > 0)
      if (gps.encode(Serial1.read()))
      {
        if (gps.location.isValid())
        {
          gps_lat = gps.location.lat();
          gps_lon = gps.location.lng();
          gps_status = 1;
        }
        else
        {
        }
      }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      // GPS DEV_ERROR
      gps_status = -1;
      Serial.println(F("No GPS detected: check wiring."));
    }

    // if (gps_status == 1)
    // {
    data_str.concat(String(FALL_STATUS));
    data_str.concat("E");
    data_str.concat(String(gps_status, 1));
    data_str.concat("E");
    data_str.concat(String(gps_lat, 6));
    data_str.concat("E");
    data_str.concat(String(gps_lon, 6));
    data_str.concat("E");
    data_str.concat(String(temp, 2));
    data_str.concat("E");
    data_str.concat(String(myBPM, 2));
    data_str.replace(".", "D");
    data_str.replace("-", "C");

    LoRa.beginPacket(); //Send LoRa packet to receiver
    LoRa.print(data_str);
    LoRa.endPacket();
    Serial.println(data_str);
    FALL_STATUS = 0;
    ESP.restart();
    // }
  }
}
int read_ACCELL_GYRO()
{
  /**
   * Read Sensor Register Values
  */

  IMU.readSensor();

  float accel_x = roundf(IMU.getAccelX_mss() + accel_x_offset);
  float accel_y = roundf(IMU.getAccelY_mss() + accel_y_offset);
  float accel_z = roundf(IMU.getAccelZ_mss() + accel_z_offset);

  // Acceleration Signal Magnitude vector
  float ASMV = sqrtf(powf(accel_x, 2) + powf(accel_y, 2) + powf(accel_z, 2));

  /**
 * FALL DETECTION ALGORITHM
 * For Every pass check if the ASMV is above threshold
 * If not return
 * Otherwise read gyro values and calculate the RSMV
 * Wait 0.5 seconds read gyro values and calculate the RSMV again
 * If no change in rotational accelation check if value above threshold
 * If value above threshold after 0.5 seconds, fall detected otherwise return
*/
  if (ASMV < ACCEL_FT)
    return 0;

  //Read gyro
  float gyro_x = roundf(IMU.getGyroX_rads() * rad2Deg);
  float gyro_y = roundf(IMU.getGyroY_rads() * rad2Deg);
  float gyro_z = roundf(IMU.getGyroZ_rads() * rad2Deg);

  // Rotational signal magnitude vector
  float RSMV1 = sqrtf(powf(gyro_x, 2) + powf(gyro_y, 2) + powf(gyro_z, 2));
  delay(500); // wait 0.5 seconds

  //read gyro
  gyro_x = roundf(IMU.getGyroX_rads() * rad2Deg);
  gyro_y = roundf(IMU.getGyroY_rads() * rad2Deg);
  gyro_z = roundf(IMU.getGyroZ_rads() * rad2Deg);

  // Rotational signal magnitude vector
  float RSMV2 = sqrtf(powf(gyro_x, 2) + powf(gyro_y, 2) + powf(gyro_z, 2));
  float R_DIFF = abs(RSMV1 - RSMV2); //adjust to fit your need

  if (RSMV2 < ROT_FT)
    return 0;

  if (R_DIFF > 5)
    return 0;
  return 1;
}
