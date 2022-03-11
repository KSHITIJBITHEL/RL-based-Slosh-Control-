void dist_ctrl(double target_dist){
  /*
   * base : the base line for the sine curve 
   * h : amplitude of sine curve 
   * phi : y offset for sine curve 
   * lambda : wavelength for sine curve 
   */
//  static int dir= 1,prev_dir=-1;
  static double dist_error, prev_dist_error, d_dist_error,dist_ctrl_op=0;
  static double base = 700, h = 200, phi = 700, lambda = 1100 ; 
//  if(abs(dist1-prev_distx)<15){
//    act_distx = dist1*cos(yaw_reading*PI/180);
//    prev_distx = dist1;
//  }
  act_distx = dist1*cos(yaw_reading*PI/180);
  target_dist = base - dir*h*sin(2*PI*(act_disty- phi )/lambda); 
  Serial.print('\t');
  Serial.print(act_disty);
  Serial.print('\t');
  Serial.println(target_dist);
  dist_error = target_dist - act_distx ;
  d_dist_error = dist_error - prev_dist_error ; 
  dist_ctrl_op =kp_d* dist_error + kd_d*d_dist_error;
  dist_ctrl_op = constrain(dist_ctrl_op,-50,50);

  prev_dist_error = dist_error;
  pwm_front_right += -dist_ctrl_op;
  pwm_front_left += dist_ctrl_op;
  pwm_back_right += dist_ctrl_op;
  pwm_back_left += -dist_ctrl_op;
}
