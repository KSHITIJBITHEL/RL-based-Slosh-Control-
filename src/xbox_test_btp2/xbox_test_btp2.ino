//--------------------------
#include <XBOXUSB.h>
//--------------------------
// For MPU
#include "I2Cdev.h"
#include "MPU9250_9Axis_MotionApps41.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//-------------------------

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#define motor_front_right 7
#define motor_front_left 8
#define motor_back_right 28
#define motor_back_left 29

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
USB Usb;
XBOXUSB Xbox(&Usb);
int pwm_front_right, pwm_front_left;
int pwm_back_right, pwm_back_left, k = 10;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 3  // use pin 3 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float f_mag[3];         // magnetometer data after conversion according to datasheet
float d_mag[4];
bool debug = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat v_mag;      // [x, y, z]            magnetic field vector
int16_t mag[3];         // raw magnetometer data
Quaternion q_mag;       // auxiliary quaternion of the magnetometer
VectorFloat gravity;
float ypr[3];           // yaw pitch roll readings
float yaw_offset, pitch_offset, roll_offset;
float yaw_reading, pitch_reading, roll_reading;
float desired_yaw;
long t = 0;
float pid = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

// initialize serial communication(Baud Rate can be changed)
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif

  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    if (debug == 1) Serial.println(packetSize);
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    if (debug == 1) {Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    }
  }
    
  if (Usb.Init() == -1) {
    if (debug == 1) Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  pinMode(motor_front_right, OUTPUT);
  pinMode(motor_front_left, OUTPUT);
  pinMode(motor_back_right, OUTPUT);
  pinMode(motor_back_left, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  calibrate();
  t = 0;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }
  got_intr();
  yaw_reading = (ypr[0] - yaw_offset) * 180 / M_PI;
  pitch_reading = (ypr[1] - pitch_offset) * 180 / M_PI;
  roll_reading = (ypr[2] - roll_offset) * 180 / M_PI;
  //    print_ypr();
//  print_reading();

  if((millis()-t) > 10){
  pid = cal_pid(desired_yaw, yaw_reading, 1, 0, 0);
  t = millis();
  }
  
  xbox_control(pid);
//  print_pwm();
  send_motor_pwm();
}

void send_motor_pwm(){
  analogWrite(motor_front_right,pwm_front_right);
  analogWrite(motor_front_left,pwm_front_left);
  analogWrite(motor_back_right,pwm_back_right);
  analogWrite(motor_back_left,pwm_back_left);

}
