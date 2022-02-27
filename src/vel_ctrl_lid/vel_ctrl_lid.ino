/*
This program is the interpretation routine of standard output protocol of TFmini-Plus product on Arduino.
For details, refer to Product Specifications.
For Arduino boards with only one serial port like UNO board, the function of software visual serial port is to be used.
*/
//#include <SoftwareSerial.h> //header file of software serial port
//SoftwareSerial Serial3(2, 3); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX
double dist, prev_dist; //actual distance measurements of LiDAR 
int strength; //signal strength of LiDAR
long timer;
float temprature, vel = 0;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package 
void setup() {
  Serial.begin(115200); //set bit rate of serial port connecting Arduino with computer
  Serial3.begin(115200); //set bit rate of serial port connecting LiDAR with Arduino
  Serial.println("start");
  delay(1000);
  Serial3.write((uint8_t)0x5A);
  Serial3.write((uint8_t)0x05);
  Serial3.write((uint8_t)0x05);
  Serial3.write((uint8_t)0x06);
  Serial3.write((uint8_t)0x6A);
  Serial3.write((uint8_t)0x5A);
  Serial3.write((uint8_t)0x04);
  Serial3.write((uint8_t)0x11);
  Serial3.write((uint8_t)0x6F);
  Serial.println("Measurement unit set..");
  timer = millis();
}
void loop() {
  if (Serial3.available()) { //check if serial port has data input
//  Serial.println("here1");
  if(Serial3.read() == HEADER) { //assess data package frame header 0x59 
    uart[0]=HEADER;
    if (Serial3.read() == HEADER) { //assess data package frame header 0x59
    uart[1] = HEADER;
    for (i = 2; i < 9; i++) { //save data in array
    uart[i] = Serial3.read(); 
    }
    check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7]; 
    if (uart[8] == (check & 0xff)){ //verify the received data as per protocol
      dist = uart[2] + uart[3] * 256; //calculate distance value 
      strength = uart[4] + uart[5] * 256; //calculate signal strength value 
      temprature = uart[6] + uart[7] *256;//calculate chip temprature 
      temprature = temprature/8 - 256;
//      Serial.print(millis()-timer);
      Serial.print(100*(dist-prev_dist));
      Serial.print("\tprev_dist = ");
      Serial.print(prev_dist); //output signal strength value
      Serial.print("\t del_t = ");
      Serial.print(millis()-timer);
      vel = (float((dist-prev_dist))/float(millis()-timer));
      prev_dist = dist;
      timer = millis();
      Serial.print("\tdist = ");
      Serial.print(dist); //output measure distance value of LiDAR Serial.print('\t');
      
//      Serial.print("\t del_t = ");
//      Serial.print(temprature);
//      Serial.print(" celcius degree"); //output chip temperature of Lidar
      
      Serial.print("\t vel = ");
      Serial.println(vel);
  } }
  } }
  delay(10);
//  else Serial.println("not connected");
}
