#ifndef SENSOR_FETCH_H
#define SENSOR_FETCH_H

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GPS.h>
#include <Arduino.h>
#include <Eigen/Dense>


// -------------------- Sensor Objects --------------------
static Adafruit_BNO055 bno = Adafruit_BNO055(55);
static Adafruit_BMP3XX bmp;

// Use built-in Serial5 on Teensy 4.1 for GPS
#define GPSSerial Serial5
static Adafruit_GPS GPS(&GPSSerial);

// Mounting quaternion
static const Eigen::Quaternionf MOUNT_QUAT = Eigen::Quaternionf(0.7071f, 0.7071f, 0.0f, 0.0f);

// Global Data Variables using Eigen
static Eigen::Quaternionf BNO_QUAT = Eigen::Quaternionf::Identity();
static Eigen::Matrix3f    BNO_ROT  = Eigen::Matrix3f::Identity();
static Eigen::Vector3f    BNO_ACC, BNO_GYRO, BNO_MAG, BNO_LINACC, BNO_GRAV;
static float              BNO_TEMPERATURE = 0.0f;

static float BMP_PRESSURE = 0, BMP_TEMPERATURE = 0, BMP_ALTITUDE = 0;

static bool   GPS_FIX = false;
static float  GPS_LATITUDE = 0, GPS_LONGITUDE = 0, GPS_ALTITUDE = 0;
static char   GPS_LAT_DIR = 'N', GPS_LON_DIR = 'E';
static uint8_t GPS_HOUR = 0, GPS_MINUTE = 0, GPS_SECOND = 0;
static uint8_t GPS_DAY = 0, GPS_MONTH = 0;
static uint16_t GPS_YEAR = 0;
static float   GPS_SPEED = 0, GPS_ANGLE = 0, GPS_HDOP = 0;
static uint8_t GPS_SATS = 0;

inline void sensorsBegin() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();

  // BNO055
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055");
    while (1);
  }
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  // BMP390
  if (!bmp.begin_I2C()) {
    Serial.println("Failed to initialize BMP390");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // GPS
  GPSSerial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
}

inline void updateBNO055() {
  imu::Quaternion qb = bno.getQuat();
  Eigen::Quaternionf q_body(qb.w(), qb.x(), qb.y(), qb.z());
  BNO_QUAT = MOUNT_QUAT * q_body;
  BNO_ROT  = BNO_QUAT.toRotationMatrix();

  // Helper lambda to read and rotate vectors
  auto readRot = [&](auto type) {
    imu::Vector<3> vr = bno.getVector(type);
    Eigen::Vector3f v(vr.x(), vr.y(), vr.z());
    return BNO_ROT * v;
  };

  BNO_ACC    = readRot(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  BNO_GYRO   = readRot(Adafruit_BNO055::VECTOR_GYROSCOPE);
  BNO_MAG    = readRot(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  BNO_LINACC = readRot(Adafruit_BNO055::VECTOR_LINEARACCEL);
  BNO_GRAV   = readRot(Adafruit_BNO055::VECTOR_GRAVITY);

  BNO_TEMPERATURE = bno.getTemp();
}

inline bool updateBMP390() {
  if(!bmp.performReading()) return false;
  BMP_PRESSURE=bmp.pressure;
  BMP_TEMPERATURE=bmp.temperature;
  BMP_ALTITUDE=bmp.readAltitude(1013.25F);
  return true;
}

inline bool updateGPS() {
  GPS.read();
  if(!GPS.newNMEAreceived() || !GPS.parse(GPS.lastNMEA())) return false;
  GPS_FIX=GPS.fix;
  GPS_LATITUDE=GPS.latitude;
  GPS_LAT_DIR=GPS.lat;
  GPS_LONGITUDE=GPS.longitude;
  GPS_LON_DIR=GPS.lon;
  GPS_ALTITUDE=GPS.altitude;
  GPS_HOUR=GPS.hour;
  GPS_MINUTE=GPS.minute;
  GPS_SECOND=GPS.seconds;
  GPS_DAY=GPS.day;
  GPS_MONTH=GPS.month;
  GPS_YEAR=GPS.year+2000;
  GPS_SPEED=GPS.speed;
  GPS_ANGLE=GPS.angle;
  GPS_SATS=GPS.satellites;
  GPS_HDOP=GPS.HDOP;
  return true;
}



#endif // SENSOR_FETCH_H
