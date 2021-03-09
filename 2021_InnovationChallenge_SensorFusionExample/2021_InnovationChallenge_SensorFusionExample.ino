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
unsigned long timer, timer_before;
int delta_t;
double * pry; //setup array pry to store data
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
}
 
void loop() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);

  //Display the results (acceleration is measured in m/s^2)
 
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");
  timer = millis()*1e-3; //return elapsed time since arduino started running sketch in seconds
  delta_t = timer-timer_before;
  pry = FusionPRY(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 0); //returns array [roll, pitch, yaw] with index [0, 1, 2]
  Serial.println(pry[1]);
  
  
  delayMicroseconds(1000);
}
