/*
 * Arduino Sensor Lightweight sensor fusion library
 * Dependencies:
 *  -- Arduino math library (For the two atan functions in radians that I haven't gotten rid of yet)
 *  -- SpeedyTrig library (Fast trig that returns in degrees, what is mostly used to make the code run faster)
 *  -- Kalman (use the one by TKJ ELectronics)
 * Written by Ron Freeman for FRC 2021 Innovation Challenge based on KalmanFilter example code
 */
#include "SpeedTrig.h"
#include <Math.h>
#include <Kalman.h>


float gyroAngleX;
float gyroAngleY;
float gyroAngleZ;
float yaw, roll, pitch, accAngleX, accAngleY;
double t_x, t_y, t_z;
double diff_x, diff_y, diff_z;
static double a[3];
static double b[3];
static double c[3];
static double d[3];

double yaw_gyro, yaw_save, yaw_acc, compYaw;

//stuff for kalman filter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

/* run this function to setup the Kalman filter with reference plane, referece plane calculated using accelerometer data in a pretty standard way.
 *  From testing over a 30 minute period the sensor drift was ±1 degree, the kalman filter is more resistant to drift
 *  The kalman filter is a lot more accurate in the long run, but not so accurate intially
 */
double gyroYaw(double gyroZ2, double dtt, double yaw_old){
  double gyro_yaw = (yaw_old + gyroZ2*dtt); 
  double gyro_yaw_deg = gyro_yaw * 180/PI;
  return gyro_yaw_deg;
}


void setupKalman (double accX, double accY, double accZ){
  
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 28 and eq. 29
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif
    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;
  
    timer = micros();
}

double * runKalman (double accX, double accY, double accZ, double gyroX, double gyroY, double gyroZ, double dt) { //acceleration in three axis and angle in rad/s on three axis
  
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
 

    
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    double acc_yaw = atan (accZ/sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG; //get an approximation of the yaw, this is relative and needs to be calibrated
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    double acc_yaw = atan (accZ/sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG; //get an approximation of the yaw, this is relative and needs to be calibrated
  #endif
    
    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s

    
  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  
    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  
    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif
  
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;
    
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  
    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
   d[0] = kalAngleX;
   d[1] = kalAngleY;
   
   yaw_gyro = gyroYaw(gyroZ, dt, yaw_save) + 0.0057; //gyro plus an error correction value to reduce drift probably need to customize for your IMU
   yaw_save = yaw_gyro * PI/180;
   yaw_acc = atan2(accZ, sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;
   compYaw = 0.5*yaw_acc + 0.5*yaw_gyro;
   d[2] = compYaw;
   
   
   return d;
}















///OLD functions

/*
 * This function uses absolute pry angles and relative acceleromter data to get absolute acceleration
 * It's optimized and gets rid of some unneeded features so it can run faster
 * data is returned in a custom data class (Ang) that holds three double variables. Ryan made me do it
 */
double * fusionXYZ(double x, double y, double z, double roll, double pitch, double yaw){ 
  //the general approach is to break each possible angle into components, and then add all the components together
  t_x = t_x + x*SpeedTrig.cos(pitch);
  t_z = t_z + x*SpeedTrig.sin(pitch);
  t_z = t_z + z*SpeedTrig.cos(pitch);
  t_x = t_x + z*SpeedTrig.sin(pitch);
  t_y = t_y + y*SpeedTrig.cos(roll);
  t_z = t_z + y*SpeedTrig.sin(roll);
  t_y = t_y + z*SpeedTrig.cos(roll);
  t_z = t_z  + z*SpeedTrig.sin(roll);
  t_y = t_y + x*SpeedTrig.sin(yaw); // note there is some ambiguity here, this could be either x or y based on orientation, not sure how to figure out which
  t_x = t_x + x*SpeedTrig.cos(yaw);
  t_y = t_y + y*SpeedTrig.cos(yaw);
  t_y = t_y + y*SpeedTrig.sin(yaw);
  a[0] = t_x;
  a[1] = t_y;
  a[2] = t_z;
  return a;
}

/* this function calculates the angle given accelerometer and gyro values and then combines them to get a more accurate angle reading
Code based on arduino gimbal https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial
 * Leave error blank if you didn't calculate it, this isn't the same value as the one found by the GravityCal function

*/
double * fusionPRY(float GyroX, float GyroY,float GyroZ, float AccX, float AccY, float AccZ, float error){
// unit conversion
  gyroAngleX = GyroX * 180 / PI; 
  Serial.println(gyroAngleX);
  gyroAngleY = GyroY  * 180 / PI;
  accAngleX = 0;
  yaw =  yaw + GyroZ * 180 / PI;
  // Calculating Roll and Pitch from the accelerometer data these functions don't seem to be working
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + error; //this line seems to be causing an overflow error on the combination complmentary filter
  
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + error; 
  // Complementary filter - combine acceleromter and gyro angle values to get a more accurate angle
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX; //combine X acceleration and gyro this line is causing an issue
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  b[0] = roll;
  b[1] = pitch;
  b[2] = yaw;
  return b;
}
//super fast and accrate PRY coordinate finding using acceleration vectors but it only goes from 90° to -90°
double * fusionAccelPRY (double accX, double accY, float accZ, float error){
  // Calculating Roll and Pitch from the accelerometer data these functions don't seem to be working
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  c[0] = roll;
  c[1] = pitch;
  return c;
}

/* given the 3 acceleration values (make sure to use output from fusionXYZ to make sure that you are using the true x,y,z realtive to the ground) this function returns the calibration factor 
 *  derived by looking for the axis with a value closest to the gravitational constant. Then using this value you can calibrate the accelerometer or remove the effects of gravity
 */
double GravityCal(double x, double y, double z){ 
  diff_x = abs(9.807-x);
  diff_y = abs(9.807-y);
  diff_z = abs(9.807-z);
  if (diff_x>diff_y){
    if (diff_y > diff_z){
      return 9.807-z;
    }
    else {
      return 9.807-y;
    }
  }
  if (diff_x > diff_z){
    if (diff_z> diff_y){
      return 9.807-y;
    }
    else{
      return 9.807-z;
    }
  }
  else{
    return 9.807-x;
  }
}
/*trapezoidal integration function to save on some lines of code and make life easier. It returns integral of two values over a definite time interval,
*the shorter the time interval the better
*/
double trapIntegral(double oldValue, double measuredValue, int dT){ 
  return (measuredValue - oldValue)*dT/2;

}
