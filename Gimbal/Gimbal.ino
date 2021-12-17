// Tim Hire 07/2020 Gimbal Project using MPU6050 accelerometer/gyroscope module

#include <Servo.h>
#include <Wire.h>

#define MPU_register 0x68 
#define acc_sensitivity_2g 16384    //--> from the module datasheet
#define gyr_sensitivity_250 131       //--> from the module datasheet

Servo servo_bottom;
Servo servo_middle;
Servo servo_top;
int servo_bottomp = 9;
int servo_middlep = 10;
int servo_topp = 11;

float acc_X, acc_Y, acc_Z, temp, gyr_X, gyr_Y, gyr_Z;
float previousTime, currentTime, elapsedTime;
float aAngleX, aAngleY, aAngleZ;
float gAngleX, gAngleY, gAngleZ; 
float yaw, roll, pitch;
int oldY = 0;               //Allows for the gimbal to be reset during operation
int oldR = 0;
int oldP = 0;

float acc_average_mag;
int shake_counter = 0;
int frames_gone = 0;
int frame_last_count = 0;
bool counting_mode;

float ax, ay, az, tt, gx, gy, gz;
int i = 0;
char potential_reset;
int reset_switch = 7;         //Pin number for the reset switch-button


//------------------------------------------------------------------------------------------------------------------------------------------------------------
// Variables that may want to be edited

int calibration_trials = 1000;    //Number of trials for the calibration process          //2000
int mixing_percent = 97;    //Percentage of the gyroscope results used in the process     //97
int refresh_rate = 80;   //delay between updates in milliseconds                          //80
int servoCal = 78;       //Affects how much the servo moves for each increment change in the MPU6050 data      //78

//------------------------------------------------------------------------------------------------------------------------------------------------------------



void setup() {
  pinMode(reset_switch, INPUT);         //Defining the pin for the input from the reset switch
  servo_bottom.attach(servo_bottomp);  servo_middle.attach(servo_middlep);  servo_top.attach(servo_topp);  
  servo_bottom.write(90);  servo_middle.write(90);  servo_top.write(90);    //Setting all of the servos to their starting positions

  Serial.begin(9600); //Could use baudrate 115200          //Could potentially up the serial rate as is being used in other programs
  Wire.begin();
  Wire.beginTransmission(MPU_register);
  Wire.write(0x6B);
  Wire.write(0x00);           //Turns sleep off allowing for the continuous stream of data from the module
  Wire.endTransmission(true);
  
  delay(20);
  counting_mode = false;
  calibrate();                  //Calibrates the data from the MPU6050 to improve the accuracy of the readings
}



void loop() {
  //************  Section for getting the data from the MPU6050 module and trying to ascertain the orientation of the module from the readings
  Wire.beginTransmission(MPU_register);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_register, 14, true);

  acc_X = Wire.read() << 8 | Wire.read();
  acc_Y = Wire.read() << 8 | Wire.read();
  acc_Z = Wire.read() << 8 | Wire.read();
  temp  = Wire.read() << 8 | Wire.read();
  gyr_X = Wire.read() << 8 | Wire.read();
  gyr_Y = Wire.read() << 8 | Wire.read();
  gyr_Z = Wire.read() << 8 | Wire.read();

  aAngleX = ((atan(acc_Y / sqrt(sq(acc_X) + sq(acc_Z))) * 180) / PI) - ax;
  aAngleY = ((atan(-1 * acc_X / sqrt(sq(acc_Y) + sq(acc_Z))) * 180) / PI) - ay;

  previousTime = currentTime;
  currentTime = millis(); 
  elapsedTime = (currentTime - previousTime) * 0.001; 

  gAngleX = gAngleX + (gyr_X - gx) * elapsedTime;
  gAngleY = gAngleY + (gyr_Y - gy) * elapsedTime;
  gAngleZ = gAngleZ + (gyr_Z - gz) * elapsedTime; 

  roll = 0.01 * mixing_percent * (gAngleX / gyr_sensitivity_250) + 0.01 * (100 - mixing_percent) * (aAngleX / acc_sensitivity_2g);  
  pitch = 0.01 * mixing_percent * (gAngleY / gyr_sensitivity_250) + 0.01 * (100 - mixing_percent) * (aAngleY / acc_sensitivity_2g);
  yaw = gAngleZ / gyr_sensitivity_250;
  roll = roll - oldR;
  pitch = pitch - oldP;
  yaw = yaw - oldY;

  Serial.print(roll);    Serial.print(",");
  Serial.print(pitch);    Serial.print(",");
  Serial.print(yaw);  Serial.print(",");
  Serial.println(currentTime);

  servo_control();                          //Makes changes to the servos based on the data from the MPU6050 module

  //------------------ Checking to see whether a reset of the gimbal has been requested
  if (Serial.available() > 0){
    potential_reset = Serial.read();
    if (potential_reset == 'r'){
      reset();
    }
  }
  if (digitalRead(reset_switch) == HIGH){
    reset();
  }
  
  delay(refresh_rate);
  frames_gone += 1;
  //shake_reset();                 //enables the gimbal to be able to be reset by shaking it 2/3 times
}









void servo_control(){
  //**********  Section controlling the servos based on data from the module
  if (roll < 90 and roll > -90){
    servo_top.write(180 - map(roll, -servoCal, servoCal, 0, 180));   //Mapping the output from the orientation to the 0 to 180 values required by the servo
  }
  else{
    Serial.println("Error in roll");
    servo_top.write(90);
  }
  
  if (pitch < (99 * servoCal / 180) and pitch > -90){
    servo_middle.write(180 - map(pitch, -servoCal, servoCal, 0, 180));
  }
  else if (pitch > (99 * servoCal / 180) and pitch < 90){
    servo_middle.write(180 - map((99 * servoCal / 180), -servoCal, servoCal, 0, 180));           //In order to prevent the arms from crashing into each other at exteme angles
  }
  else{
    Serial.println("Error in pitch");
    servo_middle.write(90);
  }
  
  if (yaw < 90 and yaw > -90){
    servo_bottom.write(180 - map(yaw, -servoCal, servoCal, 0, 180));
  }
  else{
    Serial.println("Error in yaw");
    servo_bottom.write(90);
  }  
}


void calibrate(){
  //******  Auto-calibration of the sensors to improve accuracy every time --> perhaps the potential to auto-calibrate the sensors at regular intervals if the MPU6050 is in the same orientation for a long time
  Serial.println("Starting self-calibration process...");
  vibrate();
  while (i < (calibration_trials + 100)){
    Wire.beginTransmission(MPU_register);
    Wire.write(0x3B); //Writes to the register storing the X-axis reading
    Wire.endTransmission(false);        //Keeps the port open to allow the subsequent data to be retrieved
    Wire.requestFrom(MPU_register, 14, true);
    if (i  > 100){          //Skip the first 100 readings to allow the reading to settle before starting the average
        ax = Wire.read() << 8 | Wire.read();
        ay = Wire.read() << 8 | Wire.read();
        az = Wire.read() << 8 | Wire.read();
        tt = tt + (Wire.read() << 8 | Wire.read());
        gx = gx + (Wire.read() << 8 | Wire.read());
        gy = gy + (Wire.read() << 8 | Wire.read());
        gz = gz + (Wire.read() << 8 | Wire.read());

        ax = ax + (atan(ay / sqrt(sq(ax) + sq(az))) * 180 / PI);
        ay = ay + (atan(-1 * ax / sqrt(sq(ay) + sq(az))) * 180 / PI);
    }
    delay(2);           //stops the same reading being used
    i++;
  }
  ax = ax / calibration_trials;
  ay = ay / calibration_trials;
  gx = gx / calibration_trials;
  gy = gy / calibration_trials;
  gz = gz / calibration_trials;
  Serial.println("Calibration ended");
  vibrate();
}


void shake_reset(){
  /*Serial.print(acc_X);  Serial.print(",");
  Serial.print(acc_Y);  Serial.print(",");
  Serial.print(acc_Z);  Serial.print("      ");*/

  //Experimental section trying to reset the gimbal if the gimbal is shaken twice
  //Shake defined as the average magnitude of the accelerometer readings from the MPU6050 being greater than 10,000
  //Potentially need to identify whether there is a sign change by storing the sign of the values as a binary number

  if (counting_mode == true){
    if (frames_gone - frame_last_count > 15){
      counting_mode = false;
      reset();
    }
  }

  else{
    acc_average_mag = (abs(acc_X) + abs(acc_Y) + abs(acc_Z)) / 3;
    if (acc_average_mag > 9000){
      if (frames_gone - frame_last_count < 8){
        shake_counter += 1;
        if (shake_counter > 1){              //attempting to make sure that the number of frames changes as the refresh_rate changes
          counting_mode = true;
          shake_counter = 0;
          frame_last_count = frames_gone;
        }
      }
      else{
        frame_last_count = frames_gone;       //Stores the starting frame from when the first hint of a shake is detected
      }
    }
  }
}


void reset(){             //Resets the zero position of the positioning if 'r' is in the Serial port or the push switch is pressed on the board
  servo_bottom.write(90);
  servo_middle.write(90);
  servo_top.write(90);
  oldR += roll;
  oldP += pitch;
  oldY += yaw;
}


void vibrate(){             //Assumes the vibrate function is called after the servos are all reset into the middle
  for (int j = 0; j <= 1; j++){
    servo_middle.write(110);
    servo_bottom.write(110);
    delay(60);
    servo_middle.write(90);
    servo_bottom.write(90);
    delay(200);
  }
}
