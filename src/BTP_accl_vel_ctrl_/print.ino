void print_pwm(){
  if(!flag_print)
    return;
  Serial.print(pwm_front_right);
  Serial.print(" ");
  Serial.print(pwm_front_left);
  Serial.print(" ");
  Serial.print(pwm_back_right);
  Serial.print(" ");
  Serial.print(pwm_back_left);
  Serial.print(" ");
  Serial.println();
}
void print_gyro(){
    if(!flag_print)
      return;
    Serial.print("ypr\t");
    Serial.print(yaw_reading);
    Serial.print("\t");
    Serial.print(pitch_reading);
    Serial.print("\t");
    Serial.println(roll_reading);
}
void print_gyro_offsets(){
    /*if(!flag_print)
      return;
    Serial.print("Offsets : ");
    Serial.print(yaw_offset);
    Serial.print(" ");
    Serial.print(pitch_offset);
    Serial.print(" ");
    Serial.println(roll_offset);
    */
    Serial.print("ax: ");
    Serial.print(acc_x_reading);
    Serial.print(" ");
    Serial.print("ay: ");
    Serial.print(acc_y_reading);
    Serial.print(" ");
}
void print_vel(){
    Serial.print("vx: ");
    Serial.print(vel_x);
    Serial.print("\t");
    Serial.print("vy: ");
    Serial.print(vel_y);
    Serial.print("\t");
    Serial.print("vmag: ");
    Serial.print(vel_mag);
    Serial.print(" ");
}
void print_acc(){
    Serial.print("\tax: ");
    Serial.print(acc_x_reading);
    Serial.print("\t");
    Serial.print("ay: ");
    Serial.print(acc_y_reading);
    Serial.print("\taz: ");
    Serial.print(acc_z_reading);
}
void print_lidar(){
//    if(!flag_print)
//      return;
//      Serial.print("dist1 = ");
      Serial.print(dist1); //output measure distance value of LiDAR Serial.print('\t');
//      Serial.print("strength1 = ");
//      Serial.print(strength1); //output signal strength value
//      Serial.print("dist2 = ");
      Serial.print(",  ");
      Serial.println(dist2); //output measure distance value of LiDAR Serial.print('\t');
//      Serial.print("strength2 = ");
//      Serial.println(strength2); //output signal strength value
}
