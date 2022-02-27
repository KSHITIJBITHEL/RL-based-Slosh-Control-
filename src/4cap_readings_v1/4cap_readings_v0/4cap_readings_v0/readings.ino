//---returns Capacitance values in pf---//
float read_cap(int channel,int &capdac,uint16_t *value){
  FDC.configureMeasurementSingle(MEASURMENT, channel, capdac);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_100HZ);
  
  //wait for completion
  delay(15);
  
  if (! FDC.readMeasurement(MEASURMENT, value))
  {
    int16_t msb = (int16_t) value[0];
    int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
    capacitance /= 1000;   //in femtofarads
    capacitance += ((int32_t)3028) * ((int32_t)capdac);

    if (msb > UPPER_BOUND)               // adjust capdac accordingly
  {
      if (capdac < FDC1004_CAPDAC_MAX)
    capdac++;
    }
  else if (msb < LOWER_BOUND){
      if (capdac > 0)
    capdac--;
    }
    return (float)capacitance/1000;
  }
  return 0;
}
double moving_average(double data){
  const int arr_size = 20;
  static double arr[arr_size],sum =0;
  static int index =0,n=0;
  index = index%arr_size;
  sum -= arr[index];
  arr[index] = data;
  sum += arr[index];
  index++;
  if(n<arr_size) n++;
  return sum/n;
}
