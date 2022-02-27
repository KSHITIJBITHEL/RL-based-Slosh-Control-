//////////////////////////////////////////////////////////////////////////////////////////
//
//    Code for the FDC1004 capacitance sensor breakout board
//    Author: Kshitij Bithel 
//    The code measures raw capacitance across {CHANNEL1, CHANNEL2, CHANNEL3, CHANNEL4}and Gnd 
//    To Do : Convert capacitance values to mm for scale
//    For information on how to use the board, visit https://github.com/protocentral/ProtoCentral_fdc1004_breakout
/////////////////////////////////////////////////////////////////////////////////////////
//#define Wire Wire2
#include <Wire.h>
#include <Protocentral_FDC1004.h>
#include <RunningMedian.h>



#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define MEASURMENT 0                       // measurment channel

int capdac1 = 0,capdac2 = 0,capdac3 = 0,capdac4 = 0;
uint16_t value1[2],value2[2],value3[2],value4[2];
float cap1,cap2,cap3,cap4, x, y, h;
float filcap1,filcap2,filcap3,filcap4;
float cap1_offset,cap2_offset,cap3_offset,cap4_offset;
RunningMedian filter1 = RunningMedian(10);
RunningMedian filter2 = RunningMedian(10);
RunningMedian filter3 = RunningMedian(10);
RunningMedian filter4 = RunningMedian(10);
RunningMedian filter5 = RunningMedian(5);

FDC1004 FDC;
long timer;
void setup()
{
  Wire.begin();        //i2c begin
  calibrate();
  Serial.begin(115200); // serial baud rate
}

void loop()
{
    //--- Read Capacitance Values ---//
    cap1 = read_cap(3,capdac1,value1) - cap1_offset;
    filter1.add(cap1);
    filcap1 = filter1.getMedian();
    cap2 = read_cap(2,capdac2,value2) - cap2_offset;
    filter2.add(cap2);
    filcap2 = filter2.getMedian();
    cap3 = read_cap(1,capdac3,value3) - cap3_offset;
    filter3.add(cap3);
    filcap3 = filter3.getMedian();
    cap4 = read_cap(0,capdac4,value4) - cap4_offset;
    filter4.add(cap4);
    filcap4 = filter4.getMedian();

    level();

//    cap_print();
//    cap_plot();
//    delay(10);
//    Serial.println();
}
