#include <Wire.h>

#define MPU_register 0x68     //I2C address for reseting the module to begin communication

#define acc_sensitivity_2g 16384    //--> from the module datasheet
#define gyr_sensitivity_250 131       //--> for some reason not working

float acc_X, acc_Y, acc_Z, temp, gyr_X, gyr_Y, gyr_Z;
float previousTime, currentTime, elapsedTime;
float aAngleX, aAngleY, aAngleZ;  //Calculated angles from the accelerometer by formular below --> uses trig and the constant direction of gravity
float gAngleX, gAngleY, gAngleZ; //The angle estimated by the gyroscope by multiplying the time by the angular velocity
float yaw, roll , pitch;

char userInput;    //Only prints the data if reqested
int i, iterations;    //For use during the calibration of the sensor for the first time
float ax, ay, az, tt, gx, gy, gz;   //Variables for calibrating the sensor


void setup() {
  Serial.begin(9600);
  Wire.begin();                        //Initialises Wire library
  Wire.beginTransmission(MPU_register);    //Starts communication with the module by resetting
  Wire.write(0x6B); //Talks to the register 6B (PWR_MGMT_1)
  Wire.write(0x00);  //Reset the module by placing 00 into the 6B register --> think diables the sleep mode allowing for a constant stream of readings from the module
  Wire.endTransmission(true); //Ending the transmission

  /*Wire.beginTransmission(MPU_register);   //Starts communication
  Wire.write(0x1C);       //ACCEL_CONFIG register
  Wire.write(0x10);     //chooses AFS_SEL 2 = +/- 8g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_register);
  Wire.write(0x1B);    //GYRO_CONFIG register
  Wire.write(0x10); //Chooses 1000 degrees per second
  Wire.endTransmission(true);*/
  
  //get_error();            //need this commented otherwise the graphing code does not wait
  delay(20); //Ensures that everything initialises correctly  
}


void loop() {
  if (Serial.available() > 0){
    userInput = Serial.read();
    if (userInput == 'c'){
      get_data();
    }
    if (userInput == 'i'){
      get_error();
    }
    if (userInput == 'f'){
      while (true){
        get_data();       //Emulating being a void loop() operating as normal
      }
    }
    if (userInput == 'a'){
      Wire.beginTransmission(MPU_register);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_register, 6, true);
      acc_X = Wire.read() << 8 | Wire.read();
      acc_Y = Wire.read() << 8 | Wire.read();
      acc_Z = Wire.read() << 8 | Wire.read();
      Serial.print(acc_X / acc_sensitivity_2g);      Serial.print(",");
      Serial.print(acc_Y / acc_sensitivity_2g);      Serial.print(",");
      Serial.print(acc_Z / acc_sensitivity_2g);      Serial.print(",");
      Serial.println(millis());
    }
    if (userInput == 't'){
      Wire.beginTransmission(MPU_register);
      Wire.write(41);
      Wire.write(42);
      Wire.endTransmission();
      Wire.requestFrom(MPU_register, 2, true);
      temp = (Wire.read() << 8 | Wire.read()) / 340 + 36.53;
      Serial.print(temp);      Serial.print(",");
      Serial.println(millis());
    }
    if (userInput == 'g'){
      Wire.beginTransmission(MPU_register);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_register, 6, true);
      gyr_X = Wire.read() << 8 | Wire.read();
      gyr_Y = Wire.read() << 8 | Wire.read();
      gyr_Z = Wire.read() << 8 | Wire.read();
      Serial.print(gyr_X / gyr_sensitivity_250);      Serial.print(",");
      Serial.print(gyr_Y / gyr_sensitivity_250);      Serial.print(",");
      Serial.print(gyr_Z / gyr_sensitivity_250);      Serial.print(",");
      Serial.println(millis());
    }
    if (userInput == ' '){
      Serial.println(' ');
    }
  }
}


void get_data() {
  //Getting data from the accelerometer
  Wire.beginTransmission(MPU_register);
  Wire.write(0x3B); //Writes to the register storing the X-axis reading
  Wire.endTransmission(false);        //Keeps the port open to allow the subsequent data to be retrieved
  Wire.requestFrom(MPU_register, 14, true);

  acc_X = Wire.read() << 8 | Wire.read();   //Stores the two values of the axis as one reading one after the other in 16-bit form
  acc_Y = Wire.read() << 8 | Wire.read();
  acc_Z = Wire.read() << 8 | Wire.read();
  temp  = Wire.read() << 8 | Wire.read();
  gyr_X = Wire.read() << 8 | Wire.read();
  gyr_Y = Wire.read() << 8 | Wire.read();
  gyr_Z = Wire.read() << 8 | Wire.read();

  aAngleX = ((atan(acc_Y / sqrt(sq(acc_X) + sq(acc_Z))) * 180) / PI) - 8.76;   //Gives the current angle in degrees          //These values are not at all right and are giving nan as the output
  aAngleY = ((atan(-1 * acc_X / sqrt(sq(acc_Y) + sq(acc_Z))) * 180) / PI); //Own error constants from below
  //Cannot give a proper estimate of the yaw value

  //Read time for dealing with the gyroscope data
  previousTime = currentTime;
  currentTime = millis();    //Time after the program was first initialised
  elapsedTime = (currentTime - previousTime) * 0.001; //Need to make sure in seconds as angular velocity in degrees per second

  gAngleX = gAngleX + (gyr_X + 617) * elapsedTime;
  gAngleY = gAngleY + (gyr_Y - 354) * elapsedTime; //Effectively integrating the angular velocity to work out the angle
  gAngleZ = gAngleZ + (gyr_Z - 264) * elapsedTime;   //Own error constants

  roll = 0.96 * (gAngleX / gyr_sensitivity_250) + 0.04 * (aAngleX / acc_sensitivity_2g);   //Filters both of the data to create a better angle as gyroscopes drift over time
  pitch = 0.96 * (gAngleY / gyr_sensitivity_250) + 0.04 * (aAngleY / acc_sensitivity_2g);
  yaw = gAngleZ / gyr_sensitivity_250;                              //Cannot properly work out the yaw

  Serial.print(roll);    Serial.print(",");
  Serial.print(pitch);    Serial.print(",");
  Serial.print(yaw);    Serial.print(",");
  Serial.println(currentTime);
}


void get_error(){
  i = 0;
  iterations = 2000;
  while (i < iterations){
    Wire.beginTransmission(MPU_register);
    Wire.write(0x3B); 
    Wire.endTransmission(false);    
    Wire.requestFrom(MPU_register, 14, true);
  
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    tt = tt + (Wire.read() << 8 | Wire.read());
    gx = gx + (Wire.read() << 8 | Wire.read());
    gy = gy + (Wire.read() << 8 | Wire.read());
    gz = gz + (Wire.read() << 8 | Wire.read());

    ax = ax + (atan(ay / sqrt(sq(ax) + sq(az))) * 180 / PI);
    ay = ay + (atan(-1 * ax / sqrt(sq(ay) + sq(az))) * 180 / PI);
    
    i++;
  }
  ax = ax / iterations;
  ay = ay / iterations;
  gx = gx / iterations;
  gy = gy / iterations;
  gz = gz / iterations;

  Serial.print(ax);  Serial.print(" ");
  Serial.print(ay);  Serial.print(" ");
  Serial.print(gx);  Serial.print(" ");
  Serial.print(gy);  Serial.print(" ");
  Serial.println(gz);  Serial.println("End of the calibrating procedure. Results are above:");
}
