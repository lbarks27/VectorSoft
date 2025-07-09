#ifndef SENSOR_FETCH_H
#define SENSOR_FETCH_H

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_GPS.h>
#include <Arduino.h>



// -------------------- Sensor Objects --------------------
static Adafruit_BNO055 bno = Adafruit_BNO055(55);
static Adafruit_BMP3XX bmp;

// Use built-in Serial5 on Teensy 4.1 for GPS
#define GPSSerial Serial5
static Adafruit_GPS GPS(&GPSSerial);

// -------------------- Mounting Quaternion (adjust to your mounting)
static const float MOUNT_QW = 0.7071f;
static const float MOUNT_QX = 0.7071f;
static const float MOUNT_QY = 0.0f;
static const float MOUNT_QZ = 0.0f;

// -------------------- Global Data Variables --------------------
static float BNO_QUAT_W = 0, BNO_QUAT_X = 0, BNO_QUAT_Y = 0, BNO_QUAT_Z = 0;
static float BNO_ROT[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
static float BNO_ACC_X = 0, BNO_ACC_Y = 0, BNO_ACC_Z = 0;
static float BNO_GYRO_X = 0, BNO_GYRO_Y = 0, BNO_GYRO_Z = 0;
static float BNO_MAG_X = 0, BNO_MAG_Y = 0, BNO_MAG_Z = 0;
static float BNO_LINACC_X = 0, BNO_LINACC_Y = 0, BNO_LINACC_Z = 0;
static float BNO_GRAV_X = 0, BNO_GRAV_Y = 0, BNO_GRAV_Z = 0;
static float BNO_TEMPERATURE = 0;

static float BMP_PRESSURE = 0, BMP_TEMPERATURE = 0, BMP_ALTITUDE = 0;

static bool  GPS_FIX = false;
static float GPS_LATITUDE = 0, GPS_LONGITUDE = 0, GPS_ALTITUDE = 0;
static char  GPS_LAT_DIR = 'N', GPS_LON_DIR = 'E';
static uint8_t GPS_HOUR = 0, GPS_MINUTE = 0, GPS_SECOND = 0;
static uint8_t GPS_DAY = 0, GPS_MONTH = 0;
static uint16_t GPS_YEAR = 0;
static float   GPS_SPEED = 0, GPS_ANGLE = 0, GPS_HDOP = 0;
static uint8_t GPS_SATS = 0;

// Quaternion multiplication: qc = qm * qb
inline void quatMultiply(float qw, float qx, float qy, float qz,
                         float rw, float rx, float ry, float rz,
                         float &ow, float &ox, float &oy, float &oz) {
  ow = qw*rw - qx*rx - qy*ry - qz*rz;
  ox = qw*rx + qx*rw + qy*rz - qz*ry;
  oy = qw*ry - qx*rz + qy*rw + qz*rx;
  oz = qw*rz + qx*ry - qy*rx + qz*rw;
}

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
  float bw=qb.w(), bx=qb.x(), by=qb.y(), bz=qb.z();
  float cw,cx,cy,cz;
  quatMultiply(MOUNT_QW,MOUNT_QX,MOUNT_QY,MOUNT_QZ,
               bw,bx,by,bz,
               cw,cx,cy,cz);
  BNO_QUAT_W=cw; BNO_QUAT_X=cx; BNO_QUAT_Y=cy; BNO_QUAT_Z=cz;
  float ww=cw*cw, xx=cx*cx, yy=cy*cy, zz=cz*cz;
  float wx=cw*cx, wy=cw*cy, wz=cw*cz;
  float xy=cx*cy, xz=cx*cz, yz=cy*cz;
  BNO_ROT[0][0]=ww+xx-yy-zz;
  BNO_ROT[0][1]=2*(xy-wz);
  BNO_ROT[0][2]=2*(xz+wy);
  BNO_ROT[1][0]=2*(xy+wz);
  BNO_ROT[1][1]=ww-xx+yy-zz;
  BNO_ROT[1][2]=2*(yz-wx);
  BNO_ROT[2][0]=2*(xz-wy);
  BNO_ROT[2][1]=2*(yz+wx);
  BNO_ROT[2][2]=ww-xx-yy+zz;
  imu::Vector<3> v;
  v=bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  BNO_ACC_X=BNO_ROT[0][0]*v.x()+BNO_ROT[0][1]*v.y()+BNO_ROT[0][2]*v.z();
  BNO_ACC_Y=BNO_ROT[1][0]*v.x()+BNO_ROT[1][1]*v.y()+BNO_ROT[1][2]*v.z();
  BNO_ACC_Z=BNO_ROT[2][0]*v.x()+BNO_ROT[2][1]*v.y()+BNO_ROT[2][2]*v.z();
  v=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  BNO_GYRO_X=BNO_ROT[0][0]*v.x()+BNO_ROT[0][1]*v.y()+BNO_ROT[0][2]*v.z();
  BNO_GYRO_Y=BNO_ROT[1][0]*v.x()+BNO_ROT[1][1]*v.y()+BNO_ROT[1][2]*v.z();
  BNO_GYRO_Z=BNO_ROT[2][0]*v.x()+BNO_ROT[2][1]*v.y()+BNO_ROT[2][2]*v.z();
  v=bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  BNO_MAG_X=BNO_ROT[0][0]*v.x()+BNO_ROT[0][1]*v.y()+BNO_ROT[0][2]*v.z();
  BNO_MAG_Y=BNO_ROT[1][0]*v.x()+BNO_ROT[1][1]*v.y()+BNO_ROT[1][2]*v.z();
  BNO_MAG_Z=BNO_ROT[2][0]*v.x()+BNO_ROT[2][1]*v.y()+BNO_ROT[2][2]*v.z();
  v=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  BNO_LINACC_X=BNO_ROT[0][0]*v.x()+BNO_ROT[0][1]*v.y()+BNO_ROT[0][2]*v.z();
  BNO_LINACC_Y=BNO_ROT[1][0]*v.x()+BNO_ROT[1][1]*v.y()+BNO_ROT[1][2]*v.z();
  BNO_LINACC_Z=BNO_ROT[2][0]*v.x()+BNO_ROT[2][1]*v.y()+BNO_ROT[2][2]*v.z();
  v=bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  BNO_GRAV_X=BNO_ROT[0][0]*v.x()+BNO_ROT[0][1]*v.y()+BNO_ROT[0][2]*v.z();
  BNO_GRAV_Y=BNO_ROT[1][0]*v.x()+BNO_ROT[1][1]*v.y()+BNO_ROT[1][2]*v.z();
  BNO_GRAV_Z=BNO_ROT[2][0]*v.x()+BNO_ROT[2][1]*v.y()+BNO_ROT[2][2]*v.z();
  BNO_TEMPERATURE=bno.getTemp();
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
