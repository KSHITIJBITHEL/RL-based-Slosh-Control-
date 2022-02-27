/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>
#include <RunningMedian.h>

VL53L0X sensor;
RunningMedian filterVel = RunningMedian(12);

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
//  for(int i = 0;i<12;i++) moving_average(sensor.readRangeContinuousMillimeters());
}

void loop()
{
//  moving_average(sensor.readRangeContinuousMillimeters());
  Serial.print(moving_average(sensor.readRangeContinuousMillimeters()));
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
}

float moving_average(float data){
  const int arr_size = 20;
  static float arr[arr_size],sum =0;
  static int index =0,n=0;
  index = index%arr_size;
  sum -= arr[index];
  arr[index] = data;
  sum += arr[index];
  index++;
  if(n<arr_size) n++;
  return sum/n;
}
