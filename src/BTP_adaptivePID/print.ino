void print_pwm(){
  if(!flag_print)
    return;
  Serial.print(" \t");
  Serial.print(pwm_front_right);
  Serial.print(" ");
  Serial.print(pwm_front_left);
  Serial.print(" ");
  Serial.print(pwm_back_right);
  Serial.print(" ");
  Serial.print(pwm_back_left);
  Serial.print(" ");
//  Serial.println();
}
void print_gyro(){
    if(!flag_print)
      return;
    Serial.print("ypr\t");
    Serial.println(yaw_reading);
//    Serial.print("\t");
//    Serial.print(pitch_reading);
//    Serial.print("\t");
//    Serial.println(roll_reading);
}
void print_gyro_offsets(){
    if(!flag_print)
      return;
    Serial.print("Offsets : ");
    Serial.print(yaw_offset);
    Serial.print(" ");
    Serial.print(pitch_offset);
    Serial.print(" ");
    Serial.println(roll_offset);
}
void print_lidar(){
//    if(!flag_print)
//      return;
//      Serial.print("\tdist1 = ");
//      Serial.print(dist1); //output measure distance value of LiDAR Serial.print('\t');
////      Serial.print("strength1 = ");
////      Serial.print(strength1); //output signal strength value
////      Serial.print("dist2 = ");
//      Serial.print(",  ");
//      Serial.println(dist2); //output measure distance value of LiDAR Serial.print('\t');
//      Serial.print(",  ");
//      Serial.println(vel);
//      Serial.print("strength2 = ");
//      Serial.println(strength2); //output signal strength value
}
