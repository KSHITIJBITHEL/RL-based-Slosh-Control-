#include <XBOXUSB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#define motor_front_right 7
#define motor_front_left 8
#define motor_back_right 28
#define motor_back_left 29

USB Usb;
XBOXUSB Xbox(&Usb);
int pwm_front_right, pwm_front_left;
int pwm_back_right, pwm_back_left, k = 10;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);
//#if !defined(__MIPSEL__)
//  while (!Serial){
//    digitalWrite(LED_BUILTIN, HIGH); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//  }
//#endif

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  
  pinMode(motor_front_right, OUTPUT);
  pinMode(motor_front_left, OUTPUT);
  pinMode(motor_back_right, OUTPUT);
  pinMode(motor_back_left, OUTPUT);
  analogWriteFrequency(5, 490);
  analogWriteFrequency(6, 490);
  analogWriteFrequency(7, 490);
  analogWriteFrequency(8, 490);
}

void loop() {
  xbox_control();
//    pwm_front_right = 127;
//    pwm_front_left = 127;
//    pwm_back_right = 127;
//    pwm_back_left = 127;
  send_motor_pwm();
  delay(10);
}

void send_motor_pwm(){
  analogWrite(motor_front_right,pwm_front_right);
  analogWrite(motor_front_left,pwm_front_left);
  analogWrite(motor_back_right,pwm_back_right);
  analogWrite(motor_back_left,pwm_back_left);

}
