int base_pwm_ctrl(double dist,double dist_low,double dist_high){
  static int vel = 0,vel_high = 30,vel_low = 15;
  if(dist<=(dist_high+dist_low)/2 && dist>dist_low)vel = map(dist,dist_low,(dist_high+dist_low)/2,vel_low,vel_high);
  else if(dist>(dist_high+dist_low)/2 && dist<dist_high)vel = map(dist,(dist_high+dist_low)/2,dist_high,vel_high,vel_low);
  else vel = vel_low;
  return vel;
}
void cal_velocity(){
  static int dt ;
  static long int t;
  dt = millis()-t;
  t = millis();
  vel_x = acc_x_reading*dt;
  vel_y = acc_y_reading*dt;
  vel_mag = sqrt(vel_x*vel_x + vel_y*vel_y);
}
