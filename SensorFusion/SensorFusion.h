/*
 * Arduino Sensor Lightweight sensor fusion library
 * Written by Ron Freeman for FRC 2021 Innovation Challenge 
 */
struct Ang { double t_x; double t_y; double t_z;}; 
double gyroAngleX;
double gyroAngleY;
double yaw, roll, pitch, accAngleX, accAngleY;
double t_x, t_y, t_z;
double diff_x, diff_y, diff_z;
double S
/*
 * This function uses absolute pry angles and relative acceleromter data to get absolute acceleration
 * It's optimized and gets rid of some unneeded features so it can run faster
 * data is returned in a custom data class (Ang) that holds three double variables. Ryan made me do it
 */
Ang fusionXYZ(double x, double y, double z, double roll, double pitch, double yaw){ 
  //the general approach is to break each possible angle into components, and then add all the components together
  t_x = t_x + x*cos(pitch);
  t_z = t_z + x*sin(pitch);
  t_z = t_z + z*cos(pitch);
  t_x = t_x + z*sin(pitch);
  t_y = t_y + y*cos(roll);
  t_z = t_z + y*sin(roll);
  t_y = t_y + z*cos(roll);
  t_z = t_z  + z*sin(roll);
  t_y = t_y + x*sin(yaw); // note there is some ambiguity here, this could be either x or y based on orientation, not sure how to figure out which
  t_x = t_x + x*cos(yaw);
  t_y = t_y + y*cos(yaw);
  t_y = t_y + y*sin(yaw);
  Ang a{
    t_x,
    t_y,
    t_z,
  };
 return a;
}

/* this function calculates the angle given accelerometer and gyro values and then combines them to get a more accurate angle reading
Code based on arduino gimbal https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial */
Ang FusionPRY(double GyroX, double GyroY, double GyroZ, double elapsedTime, double AccX, double AccY, double AccZ, double error){
// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + error;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + error; 
  // Complementary filter - combine acceleromter and gyro angle values to get a more accurate angle
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX; //combine Y acceleration and gyro
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  Ang b {
    pitch, roll, yaw
  };
  return b;
}

/* given the 3 acceleration values (make sure to use output from fusionXYZ to make sure that you are using the true x,y,z realtive to the ground) this function returns the calibration factor 
 *  derived by looking for the axis with a value closest to the gravitational constant. Then using this value you can calibrate the accelerometer or remove the effects of gravity
 */
char isGravity(double x, double y, double z){ 
  diff_x = abs(9.807-x);
  diff_y = abs(9.807-y);
  diff_z = abs(9.807-z);
  if (diff_x>diff_y){
    if (diff_y > diff_z){
      return diff_z;
    }
    else {
      return diff_y;
    }
  }
  if (diff_x > diff_z){
    if (diff_z> diff_y){
      return diff_y;
    }
    else{
      return diff_z;
    }
  }
  else{
    return diff_x;
  }
}
/*trapezoidal integration function to save on some lines of code and make life easier. It returns integral of two values over a definite time interval,
*the shorter the time interval the better
*/
double trapIntegration(double oldValue, double measuredValue, int dT){ 
  return (measuredValue - oldValue)*dT/2;

}
