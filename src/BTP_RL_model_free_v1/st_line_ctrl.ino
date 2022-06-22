void move_straight(double target_angle){
  static int dist_low = 200,dist_high=1800, noise = 10;
  static double orientation_error, prev_error, d_orientation_error,control_output=0, prev_dir=-1, fuzzy_op;
  act_disty = dist2*cos(yaw_reading*PI/180);
//  Serial.print(act_disty);
//  Serial.print(",");
  vel_calc();
//  Serial.print(velocity);
//  Serial.print('\t');
 if(act_disty>=(dist_high) || act_disty<=dist_low){
    if(!flag_stop){
      stop_time = millis();
      flag_stop = 1;
      
      Serial.println("penalty");
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
//  target = dist_high;
  if(flag_go_back) {
    if(act_disty>300){
      if(act_disty>400)des_vel = -3;
      else des_vel = -1.5;
    }
    else if(act_disty<250)des_vel = 3;
  }
  else des_vel = v_rl;
  if(des_vel>1 && base_pwm<10) base_pwm = 12;
  else if(des_vel<-1 && base_pwm>-8) base_pwm = -10;
//  des_vel = dir*base_vel;
  fuzzy_op = 1.5*vel_fuzzy(des_vel);
//  Serial.println(fuzzy_op);
  if(incr){ base_pwm = base_pwm+ fuzzy_op;
//    Serial.println(base_pwm);
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
//  Serial.println(orientation_error);
//  Serial.print(incr);
//  Serial.print('\t');
//  Serial.print(base_pwm);
//  Serial.print('\t');
//  Serial.print(fuzzy_op);
  prev_error = orientation_error;
//  if(flag_stop){
//  pwm_front_right = 127;
//  pwm_front_left = 127;
//  pwm_back_right = 127;
//  pwm_back_left = 127;  
//  send_motor_pwm();
//  while(1);
//  }
//  else{
  pwm_front_right += base_pwm - control_output;
  pwm_front_right = constrain(pwm_front_right,0, 254);
  pwm_front_left += base_pwm + control_output;
  pwm_front_left = constrain(pwm_front_left,0, 254);
  pwm_back_right += base_pwm - control_output;
  pwm_back_right = constrain(pwm_back_right,0, 254);
  pwm_back_left += base_pwm + control_output;
  pwm_back_left = constrain(pwm_back_left,0, 254);
//}
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
