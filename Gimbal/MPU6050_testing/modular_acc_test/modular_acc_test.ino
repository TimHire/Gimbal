#include <Wire.h>

#define MPU 0x68     //I2C address for reseting the module to begin communication

#define acc_sensitivity_2g 16384 //16384    --> from the module datasheet
#define gyr_sensitivity_250 131 //131       --> for some reason not working

int acc_x_h, acc_x_l, acc_x;


void setup() {
  Serial.begin(9600);
  Wire.begin();                        //Initialises Wire library
  Wire.beginTransmission(MPU);    //Starts communication with the module by resetting
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
  delay(20); //Ensures that everything initialises correctly  
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.write(0x3C);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 2);

  if (Wire.available() <= 2){
    //acc_x_h = Wire.read();
    //acc_x_l = Wire.read();
    acc_x = (Wire.read() << 8 | Wire.read());
  }
  

  //Serial.print(acc_x_h);
  //Serial.print(" ");
  //Serial.println(acc_x_l);
  //Serial.print(" ");
  Serial.println(acc_x);

  delay(90);
}
