void move_straight(double target_angle){
  static int dist_low = 600,dist_high=1300, noise = 10,target;
  static double orientation_error, prev_error, d_orientation_error,control_output=0, prev_dir=-1, fuzzy_op;
  act_disty = dist2*cos(yaw_reading*PI/180);
//  Serial.print(act_disty);
//  Serial.print(",");
  vel_calc();
//  Serial.print(velocity);
//  Serial.print('\t');
 if(act_disty>=(dist_high+500)){
    if(!flag_stop){
      stop_time = millis();
      flag_stop = 1;
      tripcount++;
      base_vel = 0;
      Serial.println("stop");
    }
  }
/*  else if(act_disty>=dist_low && act_disty<=dist_high) {flag_stop = 0; printed = 0;}
  if(act_disty<dist_low && flag_stop) dir=1;
  if(act_disty>dist_high && flag_stop) dir= -1;
//  if(flag_auto)Serial.print(act_disty);
//  des_vel = dir*base_vel_ctrl(act_disty,dist_low,dist_high);
*/
//  if(flag_go_back) dir = -1;
//  else dir = 1;
//  if(dir == -1) target = dist_low;
//  else target = dist_high;
  target = dist_high;
  if(flag_go_back) des_vel = -3;
  else des_vel = STC(act_disty/1000.0,target/1000.0,velocity,cap2,cap3);
//  des_vel = dir*base_vel;
  fuzzy_op = vel_fuzzy(des_vel);
  if(incr){ base_pwm = base_pwm+ fuzzy_op;
//    Serial.print(base_pwm);
    }
  else base_pwm = 0;
//  Serial.print(dir);
//  Serial.print('\t');
//  Serial.print(target);
//  Serial.print('\t');
//  Serial.println(des_vel);
  orientation_error = target_angle - yaw_reading ;
  d_orientation_error = orientation_error - prev_error ; 
  control_output =kp* orientation_error +kd*d_orientation_error;
  control_output = constrain(control_output,-50,50);
//  Serial.print("\tincr");
//  Serial.print(incr);
//  Serial.print('\t');
//  Serial.print(base_pwm);
//  Serial.print('\t');
//  Serial.print(fuzzy_op);
  prev_error = orientation_error;
  pwm_front_right += base_pwm - control_output;
  pwm_front_right = constrain(pwm_front_right,0, 254);
  pwm_front_left += base_pwm + control_output;
  pwm_front_left = constrain(pwm_front_left,0, 254);
  pwm_back_right += base_pwm - control_output;
  pwm_back_right = constrain(pwm_back_right,0, 254);
  pwm_back_left += base_pwm + control_output;
  pwm_back_left = constrain(pwm_back_left,0, 254);
}
void orientation_control (){
  static double orientation_error, prev_error, d_orientation_error,control_output=0,target_angle =0,base_pwm=127;
  orientation_error = target_angle - yaw_reading ;
  d_orientation_error = orientation_error - prev_error ; 
  control_output =kp* orientation_error +kd*d_orientation_error;
  control_output = constrain(control_output,-50,50);
  prev_error = orientation_error;
  pwm_front_right = base_pwm - control_output;
  pwm_front_right = constrain(pwm_front_right,0, 254);
  pwm_front_left = base_pwm + control_output;
  pwm_front_left = constrain(pwm_front_left,0, 254);
  pwm_back_right = base_pwm - control_output;
  pwm_back_right = constrain(pwm_back_right,0, 254);
  pwm_back_left = base_pwm + control_output;
  pwm_back_left = constrain(pwm_back_left,0, 254);
}
