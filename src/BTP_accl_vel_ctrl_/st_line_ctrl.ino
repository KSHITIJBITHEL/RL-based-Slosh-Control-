void move_straight(double target_angle){
  static int dir= 1,prev_dir=-1,dist_low = 600,dist_high=2000;
  static double orientation_error, prev_error, d_orientation_error,control_output=0, act_disty;
  act_disty = dist2*cos(yaw_reading*PI/180);
  if(act_disty<=dist_low||act_disty>=dist_high){
    if(!flag_stop){
      stop_time = millis();
      flag_stop = 1;
    }
  }
  else flag_stop = 0;
  if(act_disty<dist_low && flag_stop) dir=1;
  if(act_disty>dist_high && flag_stop) dir= -1;
//  if(flag_auto)Serial.print(act_disty);
  base_pwm = base_pwm_ctrl(act_disty,dist_low,dist_high);
  orientation_error = target_angle - yaw_reading ;
  d_orientation_error = orientation_error - prev_error ; 
  control_output =kp* orientation_error +kd*d_orientation_error;
  prev_error = orientation_error;
  pwm_front_right += dir*base_pwm - control_output;
  pwm_front_left += dir*base_pwm + control_output;
  pwm_back_right += dir*base_pwm - control_output;
  pwm_back_left += dir*base_pwm + control_output;
}
