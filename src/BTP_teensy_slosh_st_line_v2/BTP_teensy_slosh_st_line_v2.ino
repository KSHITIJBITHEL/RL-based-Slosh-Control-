//===============================================================

/* code with implementation of simple straight line tracking
with a triangular velocity profile of base PWM*/

//===============================================================
#include <XBOXUSB.h>
#include <RunningMedian.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU9250_9Axis_MotionApps41.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>
#include <Protocentral_FDC1004.h>

#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define MEASURMENT 0                       // measurment channel
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define motor_front_right 7
#define motor_front_left 8
#define motor_back_right 28
#define motor_back_left 29
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 20  // use pin 3 on Arduino Uno & most boards for IMU

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

USB Usb;
XBOXUSB Xbox(&Usb);
int pwm_front_right, pwm_front_left,base_pwm = 20;
int pwm_back_right, pwm_back_left, k = 10;       
bool flag_auto = 0,flag_print = 0, flag_slosh = 0,flag_stop=0;
double kp= 3,kd = 6;
//double kp = 0.2,kd = 0;
double kp_d= 3,kd_d = 0.1;
//double kp_d = 0.5,kd_d = 0;
double kp_v= 3,kd_v = 0.1, ki_v=0;
long int t = millis(),stop_time=0, timer;
// ================================================================
// ===               Variables for Tf mini Lidar                ===
// ================================================================
double dist1,dist2, velocity, prev_ac_distx, prev_ac_disty, prev_distx, act_disty, act_distx; //actual distance measurements of LiDAR 
int strength1,strength2; //signal strength of LiDAR
float temprature1,temprature2;
RunningMedian filterVel = RunningMedian(15);
int check; //save check value
int uart1[9],uart2[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package 
// ================================================================
// ===               Variables for Protocentral                 ===
// ================================================================
int capdac1 = 0,capdac2 = 0,capdac3 = 0,capdac4 = 0;
uint16_t value1[2],value2[2],value3[2],value4[2];
float cap1,cap2,cap3,cap4, x, y,slosh_lim = 2.5, safe_slosh_lim = 1, stop_slosh_lim = 1;
//6.11 7.58 10.33 4.73
float cap1_offset = 6.11,cap2_offset = 7.58,cap3_offset = 10.33,cap4_offset = 4.73;

FDC1004 FDC(FDC1004_400HZ);
// ================================================================
// ===                MPU control/status vars                   ===
// ================================================================
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float f_mag[3];         // magnetometer data after conversion according to datasheet
float d_mag[4];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat v_mag;      // [x, y, z]            magnetic field vector
int16_t mag[3];         // raw magnetometer data
Quaternion q_mag;       // auxiliary quaternion of the magnetometer
VectorFloat gravity;
float ypr[3];           // yaw pitch roll readings
float yaw_offset, pitch_offset, roll_offset;
float yaw_reading, pitch_reading, roll_reading;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
//  Serial.begin(115200);
  Serial3.begin(115200); //to lidar2
  Serial4.begin(115200); //to lidar1
  Serial3.write((uint8_t)0x5A);
  Serial3.write((uint8_t)0x05);
  Serial3.write((uint8_t)0x05);
  Serial3.write((uint8_t)0x06);
  Serial3.write((uint8_t)0x6A);
  Serial3.write((uint8_t)0x5A);
  Serial3.write((uint8_t)0x04);
  Serial3.write((uint8_t)0x11);
  Serial3.write((uint8_t)0x6F);
//  Serial.println("Measurement unit set to MM");

//  #if !defined(__MIPSEL__)
//    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//  #endif
  if (Usb.Init() == -1) {
      Serial.print(F("\r\nOSC did not start"));
      while (1); //halt
  }
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

   Wire2.begin();        //i2c begin
   
    
  mpu.initialize();
  // load and configure the DMP
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
//    Serial.println(packetSize);
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  calibrate();
//  Serial.println("Calibrated");
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(motor_front_right, OUTPUT);
  pinMode(motor_front_left, OUTPUT);
  pinMode(motor_back_right, OUTPUT);
  pinMode(motor_back_left, OUTPUT);
  analogWriteFrequency(motor_front_right, 490);
  analogWriteFrequency(motor_front_left, 490);
  analogWriteFrequency(motor_back_right, 490);
  analogWriteFrequency(motor_back_left, 490);
  mpu.resetFIFO();
  calibrate_slosh();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //================================================================
  // to check program time
/*  static long prog_time = 0;
  Serial.println(millis()-prog_time);
//  if(millis()-prog_time>100) mpu.resetFIFO();
  prog_time = millis();*/
//==================================================================
//================================================================
// to tune vel ctrl
  /*static long prog_time = 0;
  Serial.println(millis()-prog_time);
//  if(millis()-prog_time>100) mpu.resetFIFO();
  prog_time = millis();*/
//==================================================================

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    read_lidar1();
    read_lidar2();
    slosh_xy();
    if(!(flag_stop &&((millis()-stop_time)<2000)))
      xbox_control();
    else if(flag_stop &&((millis()-stop_time)<2000)){
      pwm_front_right = 127;
      pwm_front_left = 127;
      pwm_back_right = 127;
      pwm_back_left = 127;
      flag_slosh = 1;
    }
    send_motor_pwm();
//    print_lidar();
//    delay(5);
  }
//  Serial.print(" . "+String(fifoCount)+" "+String(mpuInterrupt)+" . ");
  delay(1);
//  Serial.println("bahar aya");
  got_intr();
  yaw_reading = (ypr[0] - yaw_offset) * 180 / M_PI;
//  yaw_reading = 0;
//  pitch_reading = (ypr[1] - pitch_offset) * 180 / M_PI;
//  roll_reading = (ypr[2] - roll_offset) * 180 / M_PI;
//  print_gyro();
//  Serial.println(yaw_reading);
}

void send_motor_pwm(){
  analogWrite(motor_front_right,pwm_front_right);
  analogWrite(motor_front_left,pwm_front_left);
  analogWrite(motor_back_right,pwm_back_right);
  analogWrite(motor_back_left,pwm_back_left);

}
