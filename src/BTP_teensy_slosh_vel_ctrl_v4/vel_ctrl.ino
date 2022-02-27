int base_vel_ctrl(double dist,double dist_low,double dist_high){
  static double vel = 0,vel_high = 6,vel_low = 1;
  if(dist<=(dist_high+dist_low)/2 && dist>dist_low)vel = map(dist,dist_low,(dist_high+dist_low)/2,vel_low,vel_high);
  else if(dist>(dist_high+dist_low)/2 && dist<dist_high)vel = map(dist,(dist_high+dist_low)/2,dist_high,vel_high,vel_low);
  else vel = vel_low;
  return vel;
}

void vel_calc(){
  static double temp;
  temp = double(act_disty-prev_ac_disty)/(double(millis()-timer)/10);
//  Serial.print('\t');
//  Serial.println(millis()-timer);
//  Serial.println('\t');
  timer = millis();
  prev_ac_disty = act_disty;
//  Serial.print(temp);
//  Serial.print('\t');
  filterVel.add(temp);
  velocity = filterVel.getMedian();
}

double vel_PID(double desired_vel){
  static double vel_error, prev_vel_error, d_vel_error, i_vel_error, vel_ctrl;
  vel_error = desired_vel-velocity;
  d_vel_error = vel_error-prev_vel_error;
  if(vel_error<1)  i_vel_error += vel_error;
  else i_vel_error = 0;
  vel_ctrl = kp_v*vel_error+ki_v*i_vel_error-kd_v*d_vel_error;
  vel_ctrl = constrain(vel_ctrl, -50, 50);
  prev_vel_error = vel_error;
  return vel_ctrl;
}
