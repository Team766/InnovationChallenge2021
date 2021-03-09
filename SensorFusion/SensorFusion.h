struct Ang { double t_x; double t_y; double t_z;}; 
double gyroAngleX;
double gyroAngleY;
double yaw, roll, pitch, accAngleX, accAngleY;
Ang fusionXYZ(double x, double y, double z, double roll, double pitch, double yaw){ 
  double t_x, t_y, t_z;
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


Ang FusionPRY(double GyroX, double GyroY, double GyroZ, double elapsedTime, double AccX, double AccY, double AccZ, double error){
// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + error;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + error; 
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX; //combine Y acceleration and gyro
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  Ang b {
    pitch, roll, yaw
  };
  return b;
}

