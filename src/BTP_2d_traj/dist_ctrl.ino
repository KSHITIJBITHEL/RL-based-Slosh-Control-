void dist_ctrl(double target_dist){
  /*
   * base : the base line for the sine curve 
   * h : amplitude of sine curve 
   * phi : y offset for sine curve 
   * lambda : wavelength for sine curve 
   */
//  static int dir= 1,prev_dir=-1;
  static double dist_error, prev_dist_error, d_dist_error,dist_ctrl_op=0, sum_dist_error = 0;
  static double base = 70, h = 20, phi = 300, lambda = 1700 ; 
//  if(abs(dist1-prev_distx)<15){
//    act_distx = dist1*cos(yaw_reading*PI/180);
//    prev_distx = dist1;
//  }
  act_distx = dist1*cos(yaw_reading*PI/180);
  target_dist = base - h*sin(2*PI*(act_disty- phi )/lambda); 
  
  Serial.print(act_distx);
//  Serial.print('\t');
//  Serial.print(dir);
  Serial.print('\t');
  Serial.println(target_dist);
  dist_error = target_dist - act_distx ;
  d_dist_error = dist_error - prev_dist_error ; 
  dist_ctrl_op =kp_d* dist_error + kd_d*d_dist_error;
  dist_ctrl_op = constrain(dist_ctrl_op,-20,20);
  if(!incr){
    sum_dist_error += dist_error;
    dist_ctrl_op += ki_d*sum_dist_error;
  }
  else sum_dist_error = 0;
//  Serial.print('\t');
//  Serial.println(dist_ctrl_op);
  prev_dist_error = dist_error;
  pwm_front_right += -dist_ctrl_op;
  pwm_front_left += dist_ctrl_op;
  pwm_back_right += dist_ctrl_op;
  pwm_back_left += -dist_ctrl_op;
}
