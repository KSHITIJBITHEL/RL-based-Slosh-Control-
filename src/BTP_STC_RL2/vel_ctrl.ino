 double base_vel_ctrl(double dist,double dist_low,double dist_high){
  static double vel, temp, vel_high = 6,vel_low = 0.5;
  if(dist<=(dist_high+dist_low)/2 && dist>dist_low) vel = double(map(dist,dist_low,(dist_high+dist_low)/2,100*vel_low,100*vel_high));
  else if(dist>(dist_high+dist_low)/2 && dist<dist_high) vel = double(map(dist,(dist_high+dist_low)/2,dist_high,100*vel_high,100*vel_low));
  else vel = 100*vel_low;
  vel = vel/100;
  return vel;
}

void vel_calc(){
  if(millis()-timer>100) {
    timer = millis(); 
    prev_ac_disty = act_disty;
//    velocity =0;
    return;
    }
  static double temp=0;
  temp = double(act_disty-prev_ac_disty)/(double(millis()-timer)/10);
//  Serial.print('\t');
//  Serial.print(millis()-timer);
//  Serial.print('\t');
  timer = millis();
//  Serial.print(temp);
//  Serial.print('\t');
//  Serial.print(act_disty-prev_ac_disty);
  prev_ac_disty = act_disty;
//  filterVel.add(temp);
  if(temp<20)
  velocity = moving_avg_vel(temp);
}

double vel_PID(double desired_vel){
  vel_error = desired_vel-velocity;
  d_vel_error = vel_error-prev_vel_error;/*if(!(abs(vel_error-prev_vel_error)<2)) */
//  else d_vel_error = 0;
  i_vel_error += vel_error;/*if(abs(vel_error)<1) */
//  else i_vel_error = 0;
  vel_ctrl = kp_v*vel_error+ki_v*i_vel_error+kd_v*d_vel_error;
  vel_ctrl = constrain(vel_ctrl, -100, 100);
//  Serial.print(" ki*e ");
//  Serial.print(ki_v*i_vel_error);
//  Serial.print("\to/p ");
//  Serial.print(vel_ctrl);
  prev_vel_error = vel_error;
  return vel_ctrl;
}
double vel_fuzzy(double desired_vel){
  vel_error = desired_vel-velocity;
  d_vel_error = vel_error-prev_vel_error;/*if(!(abs(vel_error-prev_vel_error)<3))*/
  vel_ctrl = -vel.centroid_correction( vel_error, d_vel_error);;
  vel_ctrl = constrain(vel_ctrl, -100, 100);
//  Serial.print(" ki*e ");
//  Serial.print(ki_v*i_vel_error);
//  Serial.print("\t ");
//  Serial.print(vel_ctrl);
//  Serial.print("\t ");
  prev_vel_error = vel_error;
  return vel_ctrl;
}
