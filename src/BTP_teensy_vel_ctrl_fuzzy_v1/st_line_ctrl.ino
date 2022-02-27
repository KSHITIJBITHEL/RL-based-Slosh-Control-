void move_straight(double target_angle){
  static int dist_low = 600,dist_high=2000, noise = 10;
  static double orientation_error, prev_error, d_orientation_error,control_output=0, dir= 1, prev_dir=-1;
  act_disty = dist2*cos(yaw_reading*PI/180);
  vel_calc();
//  Serial.println(velocity);
  if(act_disty<=dist_low-noise||act_disty>=dist_high+noise){
    if(!flag_stop){
      stop_time = millis();
      flag_stop = 1;
    }
  }
  else if(act_disty>=dist_low && act_disty<=dist_high) flag_stop = 0;
  if(act_disty<dist_low && flag_stop) dir=1;
  if(act_disty>dist_high && flag_stop) dir= -1;
//  if(flag_auto)Serial.print(act_disty);
//  des_vel = dir*base_vel_ctrl(act_disty,dist_low,dist_high);
  des_vel = dir*base_vel;
  base_pwm = vel_fuzzy(des_vel);
//  base_pwm = 0;
//  Serial.print(dir);
  orientation_error = target_angle - yaw_reading ;
  d_orientation_error = orientation_error - prev_error ; 
  control_output =kp* orientation_error +kd*d_orientation_error;
  control_output = constrain(control_output,-50,50);
  prev_error = orientation_error;
  pwm_front_right += base_pwm - control_output;
  pwm_front_left += base_pwm + control_output;
  pwm_back_right += base_pwm - control_output;
  pwm_back_left += base_pwm + control_output;
}
