void move_straight(double target_angle){
  static int dist_low = 600,dist_high=2000, noise = 10;
  static double orientation_error, prev_error, d_orientation_error,control_output=0, dir= -1, prev_dir=1, fuzzy_op;
  act_disty = dist2*cos(yaw_reading*PI/180);
  vel_calc();
  
  if(millis()-ramptime<=2000 && flag_stop == 0){
    des_vel = (100*des_vel+2.5*dir)/100;
  }
  else if(millis()-ramptime>2000 && millis()-ramptime<=4000 && flag_stop == 0){
//    des_vel = (100*des_vel-5*dir)/100;
//  continue;
  }
  else if(millis()-ramptime>4000 && millis()-ramptime<=6000 && flag_stop == 0){
    des_vel = (100*des_vel-2.5*dir)/100;
  }
  else if(flag_stop == 0){
    flag_stop = 1;
    stop_time = millis();
    des_vel = 0;
    dir = -1*dir;
    ramptime = millis();
  }

  fuzzy_op = vel_fuzzy(des_vel);
//  Serial.print(fuzzy_op);
//  Serial.print("\t");
  if(incr) base_pwm = base_pwm+ fuzzy_op;
  else base_pwm = 0;
//    Serial.print(base_pwm);
   
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
