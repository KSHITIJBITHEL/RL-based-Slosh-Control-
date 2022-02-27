void move_straight(double target_angle){
  static int dir= 1,prev_dir=-1;
  static double orientation_error, prev_error, d_orientation_error,control_output=0, act_disty;
  act_disty = dist2*cos(yaw_reading*PI/180);
//  if(flag_auto)Serial.print(act_disty);
  vel_calc();
  Serial.println(velocity);
  if(act_disty<60){
    if(dir == -1){
    pwm_front_right = 127;
    pwm_front_left = 127;
    pwm_back_right = 127;
    pwm_back_left = 127;
    send_motor_pwm();
    dir = 1;
    delay(500);
    }
  }
   if(act_disty>150){
    if(dir == 1){
    pwm_front_right = 127;
    pwm_front_left = 127;
    pwm_back_right = 127;
    pwm_back_left = 127;
    send_motor_pwm();
    dir = -1;
    delay(500);
    }
   }
  orientation_error = target_angle - yaw_reading ;
  d_orientation_error = orientation_error - prev_error ; 
  control_output =kp* orientation_error +kd*d_orientation_error;
  prev_error = orientation_error;
  pwm_front_right += dir*base_pwm - control_output;
  pwm_front_left += dir*base_pwm + control_output;
  pwm_back_right += dir*base_pwm - control_output;
  pwm_back_left += dir*base_pwm + control_output;
}
