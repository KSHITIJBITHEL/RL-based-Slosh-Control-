void dist_ctrl(double target_dist){
//  static int dir= 1,prev_dir=-1;
  static double dist_error, prev_dist_error, d_dist_error,dist_ctrl_op=0;
//  Serial.print(abs(dist1-prev_distx));
//  Serial.print("\t");
//  Serial.println(act_distx); 
//  if(abs(dist1-prev_distx)<15){
    act_distx = dist1*cos(yaw_reading*PI/180);
    prev_distx = dist1;
//  }
//  act_distx = dist1*cos(yaw_reading*PI/180);
  
  dist_error = target_dist - act_distx ;
  d_dist_error = dist_error - prev_dist_error ; 
  dist_ctrl_op =kp_d* dist_error +kd_d*d_dist_error;
//  Serial.print(dist_ctrl_op);
//  Serial.print("\t");
//  Serial.println(act_distx);
//  Serial.print(" "); 
//  Serial.print(dist1);
//  Serial.print(" ");
  prev_dist_error = dist_error;
  pwm_front_right += -dist_ctrl_op;
  pwm_front_left += dist_ctrl_op;
  pwm_back_right += dist_ctrl_op;
  pwm_back_left += -dist_ctrl_op;
}
