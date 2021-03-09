/*
 * Arduino Sensor Lightweight sensor fusion library
 * Dependencies:
 *  -- Arduino math library (For the two atan functions in radians that I haven't gotten rid of yet)
 *  -- SpeedyTrig library (Fast trig that returns in degrees, what is mostly used to make the code run faster)
 * Written by Ron Freeman for FRC 2021 Innovation Challenge 
 */
#include "SpeedTrig.h"
#include <Math.h>



double gyroAngleX;
double gyroAngleY;
double yaw, roll, pitch, accAngleX, accAngleY;
double t_x, t_y, t_z;
double diff_x, diff_y, diff_z;
double * a;
double * b;
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
double * FusionPRY(double GyroX, double GyroY, double GyroZ, double AccX, double AccY, double AccZ, double error){
// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX * 180 / PI; 
  gyroAngleY = gyroAngleY  * 180 / PI;
  yaw =  yaw + GyroZ * 180 / PI;
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + error;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + error; 
  // Complementary filter - combine acceleromter and gyro angle values to get a more accurate angle
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX; //combine Y acceleration and gyro
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  b[0] = roll;
  b[1] = pitch;
  b[2] = yaw;
  return b;
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
double trapIntegration(double oldValue, double measuredValue, int dT){ 
  return (measuredValue - oldValue)*dT/2;

}
