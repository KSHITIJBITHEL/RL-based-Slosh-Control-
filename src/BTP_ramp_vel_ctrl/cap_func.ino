void calibrate_slosh (){
    static float sum1 = 0,sum2 = 0,sum3 = 0,sum4 = 0;
    for (int i = 0; i<100;i++){
    cap1 = read_cap(3,capdac1,value1);
    cap2 = read_cap(2,capdac2,value2);
    cap3 = read_cap(1,capdac3,value3);
    cap4 = read_cap(0,capdac4,value4); 
    sum1 += cap1;
    sum2 += cap2;
    sum3 += cap3;
    sum4 += cap4;
    }
    cap1_offset = sum1/100.0;
    cap2_offset = sum2/100.0;
    cap3_offset = sum3/100.0;
    cap4_offset = sum4/100.0;
}

//--- Prints Capacitance Values ---//
//void cap_print(){
//  Serial.print(" C1 : ");
//  Serial.print(cap1);
//  Serial.print(" C2 : ");
//  Serial.print(cap2);
//  Serial.print(" C3 : ");
//  Serial.print(cap3);
//  Serial.print(" C4 : ");
//  Serial.print(cap4);
//}

//--- Plots Capacitance Values ---//
void cap_plot(){
//  Serial.println(x);
//  Serial.print(cap1);
//  Serial.print(" ");
//  Serial.print(cap2);
//  Serial.print(" ");
//  Serial.print(cap3);
//  Serial.print(" ");
//  Serial.println(cap4);
}

//---returns Capacitance values in pf---//
float read_cap(int channel,int &capdac,uint16_t *value){
  FDC.configureMeasurementSingle(MEASURMENT, channel, capdac);
  FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ);
  
  //wait for completion
  delay(3);
  
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

void slosh_xy(){
  cap1 = read_cap(3,capdac1,value1) - cap1_offset;
  cap2 = read_cap(2,capdac2,value2) - cap2_offset;
  cap3 = read_cap(1,capdac3,value3) - cap3_offset;
  cap4 = read_cap(0,capdac4,value4) - cap4_offset;
//  delay(6);
  x = cap1-cap4;
  y = cap3-cap2;
  static double abs_slosh,avg_slosh;
  abs_slosh = sqrt(x*x+y*y);
  avg_slosh = moving_average(abs_slosh);
  if(abs_slosh>slosh_lim && flag_auto && !flag_slosh && !flag_stop) {flag_auto = 0; flag_slosh = 1;}
  else if(avg_slosh<stop_slosh_lim && flag_auto && flag_slosh) flag_slosh = 0;
  else if(avg_slosh<safe_slosh_lim && flag_slosh == 1 && !flag_auto) {flag_auto = 1;flag_slosh = 0;}
//  Serial.print(abs_slosh);
//  Serial.print("\t");
//  Serial.print(avg_slosh);
//  Serial.print("\t");
//  Serial.print(cap1);
//  Serial.print("\t");
//  Serial.print(cap2);
//  Serial.print("\t");
//  Serial.print(cap3);
//  Serial.print("\t");
//  Serial.print(cap3);
//  Serial.print("\tflag slosh = ");
//  Serial.print(flag_slosh);
//  Serial.print("\tflag stop = ");
//  Serial.println(flag_stop);
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
double moving_avg_vel(double data){
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
