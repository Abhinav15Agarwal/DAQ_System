//xxh_tesing...
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "PinChangeInterrupt.h"
#include <nRF24L01.h>
#include <RF24.h>

//NRF module Confirigation
RF24 radio(7,8);
const byte address[6] = "00001";//Unidirectional Communication


int rev1=0;
int rev2=0;
int rev3=0;
int rev4=0;
float rpm1;
float rpm2;
float rpm3;
float rpm4;
float oldtime1=0;
float oldtime2=0;
float oldtime3=0;
float oldtime4=0;
float time;

int rightf = A2;
int leftf = A1;
int rightr = A3;
int leftr = A4;
int steer = A0;
int tps = A6;
int bps = A5;

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void isr1() //interrupt service routine
{
rev1++;
}
void isr2() //interrupt service routine
{
rev2++;
}
void isr3() //interrupt service routine
{
rev3++;
}
void isr4() //interrupt service routine
{
rev4++;
}

File record;
void setup() {
  radio.begin();
  radio.openWritingPipe(address); // Calling the address
  radio.setPALevel(RF24_PA_MAX); // Putting the power level to max
  radio.setDataRate(RF24_250KBPS); // Defining the speed of the transmission
  radio.stopListening();
  
  Serial.begin(19200);
  pinMode(rightf, INPUT);
  pinMode(leftf, INPUT);
  pinMode(rightr, INPUT);
  pinMode(leftr, INPUT);
  pinMode(steer, INPUT);
  pinMode(tps, INPUT);
  pinMode(bps, INPUT);
 
  if (!SD.begin(44)) // ChipSelect
  {
    Serial.println(" SD Initialization failed!");
      }
      else{
  Serial.println("Initialization done");
  record = SD.open("daq.txt", FILE_WRITE);
      }
  if (record)
  {
    Serial.println("Writing to text file.....");
    record.print("Time(ms),Front_Right,Front_Left,Rear_Right,Rear_Left,steering,tps,bps,rpm1,rpm2,rpm3,AccX,AccY,AccZ,Pitch,Roll,Yaw,GyroX,GyroY,GyroZ");
    record.close();
  } else Serial.println("writing failed: ""daq.txt"" file not found ");

  //mpu initialization
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);1Q
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);

  //wheel speed sensor input attaching
  attachPinChangeInterrupt(4, isr1, RISING);
  attachPinChangeInterrupt(5, isr2, RISING);
  attachPinChangeInterrupt(6, isr3, RISING);
  attachPinChangeInterrupt(7, isr4, RISING);
}

void loop() {
  float rf = sen(rightf) * 30;
  float lf = sen(leftf) * 30;
  float rr = sen(rightr) * 16;
  float lr = sen(leftr) * 16;
  float st = sen(steer);
  float tp = sen(tps);
  float bp = sen(bps);

 // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
 

detachPinChangeInterrupt(7);           //detaches the interrupt
time=micros()-oldtime1;        //finds the time 
rpm1=(rev1/time)*60000000/12;         //calculates rpm
oldtime1=micros();             //saves the current time
rev1=0;
attachPinChangeInterrupt(7, isr1, RISING);

detachPinChangeInterrupt(5);           //detaches the interrupt
time=micros()-oldtime2;        //finds the time 
rpm2=(rev2/time)*60000000/12;         //calculates rpm
oldtime2=micros();             //saves the current time
rev2=0;
attachPinChangeInterrupt(5, isr2, RISING);

detachPinChangeInterrupt(4);           //detaches the interrupt
time=micros()-oldtime3;        //finds the time 
rpm3=(rev3/time)*60000000/12;         //calculates rpm
oldtime3=micros();             //saves the current time
rev3=0;
attachPinChangeInterrupt(4, isr3, RISING);

detachPinChangeInterrupt(6);           //detaches the interrupt
time=micros()-oldtime4;        //finds the time 
rpm4=(rev4/time)*60000000/12;         //calculates rpm
oldtime4=micros();             //saves the current time
rev4=0;
attachPinChangeInterrupt(6, isr4, RISING);

float kmph = ((rpm1+rpm2)/2)*0.088548;
float kmphr = ((rpm3+rpm4)/2)*0.088548;

String datastring1="";
String datastring2="";
String datastring3="";
String datastring4="";

datastring1 = String(millis())+","+String(st)+","+String(tp)+","+String(bp)+String(pitch)+'\n';

datastring2 = String(rf)+","+String(lf)+","+String(rr)+","+String(lr)+'\n';

datastring3 = String(rpm1)+","+String(rpm2)+","+String(rpm3)+","+String(rpm4)+","+String(kmph)+","+String(kmphr)+'\n';

datastring4 = String(AccX)+","+String(AccY)+","+String(AccZ)+","+String(roll)+","+String(yaw)+'\n';


Serial.print(datastring1);
Serial.print(datastring2);
Serial.print(datastring3);
Serial.print(datastring4);


  record = SD.open("daq.txt", FILE_WRITE);
  if (record)
  { 
    record.print(datastring1);
    record.print(datastring2);
    record.print(datastring3);
    record.print(datastring4);
    record.close();
  }
  //Transmitting the data
 const String text1 = datastring1;
  radio.write(&text1, sizeof(text1));

  const String text2 = datastring2;
  radio.write(&text2, sizeof(text2));

  const String text3 = datastring3;
  radio.write(&text3, sizeof(text3));

  const String text4 = datastring4;
  radio.write(&text4, sizeof(text4));
  
  delay(1000);   
}
float sen(float pin)
{
  float sensor = analogRead(pin);
  float s = (sensor * 5) / 102.3;
  return (s);
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  }
