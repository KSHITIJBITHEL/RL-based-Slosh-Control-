//////////////////////////////////////////////////////////////////////////////////////////
//
//    Code for the FDC1004 capacitance sensor breakout board
//    Author: Kshitij Bithel 
//    The code measures raw capacitance across {CHANNEL1, CHANNEL2, CHANNEL3, CHANNEL4}and Gnd 
//    To Do : Convert capacitance values to mm for scale
//    For information on how to use the board, visit https://github.com/protocentral/ProtoCentral_fdc1004_breakout
/////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <Protocentral_FDC1004.h>

#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define MEASURMENT 0                       // measurment channel
//6.11 7.58 10.33 4.73

int capdac1 = 0,capdac2 = 0,capdac3 = 0,capdac4 = 0;
long timer;
uint16_t value1[2],value2[2],value3[2],value4[2];
float cap1,cap2,cap3,cap4, x, y;
float cap1_offset= 6.11,cap2_offset = 7.58,cap3_offset = 10.33,cap4_offset = 4.73;

FDC1004 FDC(FDC1004_400HZ);
void setup()
{
  Wire.begin();        //i2c begin
  Serial.begin(115200); // serial baud rate
  calibrate();
  timer = millis();
//  print_offset();
}
//2.04  4 -0.79 0.10

void loop()
{
  static float h;
  
    //--- Read Capacitance Values ---//
//    cap1 = read_cap(0,capdac1,value1) - cap1_offset;
//    cap2 = read_cap(1,capdac2,value2) - cap2_offset;
//    cap3 = read_cap(2,capdac3,value3) - cap3_offset;
    cap4 = read_cap(3,capdac4,value4) - cap4_offset;
    cap4 = moving_average(cap4);
//    cap_print();
    cap_plot();
//    delay(0);
//    Serial.println();
}
