void dist_ctrl(double target_dist){
//  static int dir= 1,prev_dir=-1;
  static double dist_error, prev_dist_error, d_dist_error,dist_ctrl_op=0, act_distx;
  act_distx = dist1*cos(yaw_reading*PI/180);
//  if(flag_auto){ Serial.print(", ");
//  Serial.print(act_distx);
//  Serial.print("    ");
////  delayMicroseconds(100);
//  }
  
//  if(millis()-t>5500){
//    dir = -1*prev_dir;
//    prev_dir = -1*prev_dir;
//    t = millis();
//  }
//   if(millis()-t>5000){
//    dir=0;
//   }
  dist_error = target_dist - act_distx ;
  d_dist_error = dist_error - prev_dist_error ; 
  dist_ctrl_op =kp_d* dist_error +kd_d*d_dist_error;
//  Serial.print(dist2);
//  Serial.println(" ");
  prev_dist_error = dist_error;
  pwm_front_right += -dist_ctrl_op;
  pwm_front_left += dist_ctrl_op;
  pwm_back_right += dist_ctrl_op;
  pwm_back_left += -dist_ctrl_op;
}
