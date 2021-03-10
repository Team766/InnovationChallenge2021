#include <Adafruit_LSM6DSO32.h>
#include "SensorFusion.h"
int piezoPin = 8;
// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11
#define freq 100 //frequency to run code in Hz
char ground;
int velofball = 0;
int delta_t;
double i_gyroy = 0;
double i_gyrox = 0;
double i_gyroz = 0;
double accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z;
double * pr;
double * rpy;
Adafruit_LSM6DSO32 dso32;

void setup(void) {
  Serial.begin(115200);
  //setting up accelerometer
  Serial.println("Adafruit LSM6DSO32 test!");
 
  if (!dso32.begin_I2C()) {
    // if (!dso32.begin_SPI(LSM_CS)) {
    // if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSO32 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("LSM6DSO32 Found!");
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  pinMode(piezoPin, OUTPUT);

  //kalman filter stuff
  
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);
  /* Update all the values */
  accel_X = accel.acceleration.x;
  accel_Y = accel.acceleration.y;
  accel_Z = accel.acceleration.z;
  tempRaw = temp.temperature;
  gyro_X = gyro.gyro.x;
  gyro_Y = gyro.gyro.y;
  gyro_Z = gyro.gyro.z;
  
  setupKalman(accel_X, accel_Y, accel_Z);
  
}
 
void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);
  /* Update all the values */
  accel_X = accel.acceleration.x;
  accel_Y = accel.acceleration.y;
  accel_Z = accel.acceleration.z;
  tempRaw = temp.temperature;
  gyro_X = gyro.gyro.x;
  gyro_Y = gyro.gyro.y;
  gyro_Z = gyro.gyro.z;
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
  
  rpy = runKalman(accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z, dt);
  Serial.print("\n Yaw: "); Serial.print(rpy[2]);
  Serial.print("\n Roll: "); Serial.print(rpy[0]);
  Serial.print("\n Pitch: "); Serial.print(rpy[1]);
  delay(10);

}
