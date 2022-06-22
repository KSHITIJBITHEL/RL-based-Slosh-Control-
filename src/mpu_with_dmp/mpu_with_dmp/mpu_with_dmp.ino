///////////////////////////////////////////////////////////////////////////////////////
//   Arduino sketch for MPU9250 class using DMP (MotionApps v2.0)
//   I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
//   for both classes must be in the include path of your project
//   =========================================================================
//   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
//   depends on the MPU-9250's INT pin being connected to the Arduino's
//   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
//   digital I/O pin 2.
//   =========================================================================
///////////////////////////////////////////////////////////////////////////////////////


#include "I2Cdev.h"
#include "MPU9250_9Axis_MotionApps41.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// add sensor magnetometer data output using DMP
// #define OUTPUT_COMPASS

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 20  // use pin 2 on Arduino Uno & most boards and using 20 for Teensy
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
int counter = 0;
bool calib = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat v_mag;      // [x, y, z]            magnetic field vector
int16_t mag[3];         // raw magnetometer data
Quaternion q_mag;       // auxiliary quaternion of the magnetometer
VectorFloat gravity;
float ypr[3];           // yaw pitch roll readings
float yaw_offset, pitch_offset, roll_offset;
float yaw_reading, pitch_reading, roll_reading;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
  got_intr();
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication(Baud Rate can be changed)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println("start");

  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println("0");
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println("1");

  // verify connection
  // Serial.println(F("Testing device connections..."));
  // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  Serial.println("2");

  // supply your own gyro offsets here, scaled for min sensitivity
  //    mpu.setXGyroOffset(220);
  //    mpu.setYGyroOffset(76);
  //    mpu.setZGyroOffset(-85);
  //    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
    Serial.println(packetSize);
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  Serial.println("calibration started");
//  while(!calib){
//    
//  }
  calibrate();
  Serial.println("calibrated");
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
//  while (!mpuInterrupt && fifoCount < packetSize) {
//    // other program behavior stuff here
//    // .
//    // .
//    // .
//    // if you are really paranoid you can frequently test in between other
//    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
//    // while() loop to immediately process the MPU data
//    // .
//    // .
//    // .
//  }


  //    Serial.println();
  delay(20);
}
