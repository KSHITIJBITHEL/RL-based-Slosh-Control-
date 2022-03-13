 double base_vel_ctrl(double dist,double dist_low,double dist_high){
  static double vel, temp, vel_high = 6,vel_low = 0.5;
  if(dist<=(dist_high+dist_low)/2 && dist>dist_low) vel = double(map(dist,dist_low,(dist_high+dist_low)/2,100*vel_low,100*vel_high));
  else if(dist>(dist_high+dist_low)/2 && dist<dist_high) vel = double(map(dist,(dist_high+dist_low)/2,dist_high,100*vel_high,100*vel_low));
  else vel = 100*vel_low;
  vel = vel/100;
  return vel;
}

//double triangle_vel()

void vel_calc(){
  static double temp;
  temp = double(act_disty-prev_ac_disty)/(double(millis()-timer)/10);
//  Serial.print('\t');
//  Serial.println(millis()-timer);
//  Serial.println('\t');
  timer = millis();
  prev_ac_disty = act_disty;
//  Serial.print(temp);
//  Serial.print('\t');
//  filterVel.add(temp);
  if(temp<20) velocity = moving_avg_vel(temp);
}
double cal_vel_trap(int dist_low, int dist_high){
  static double des_speed, maxvel = 3,total_d = dist_high-dist_low, accl_d = 0,rate=10,dist_stop= dist_high;
 if(des_speed<maxvel && flag_stop == 0 && stepup){
    des_speed = (100*des_speed+rate)/100;
    ramptime = millis();
  }
 else if(dir == 1){
     if(act_disty<(dist_high - accl_d) && flag_stop == 0){// for dir == 1
        if(stepup) accl_d = act_disty-dist_stop;
        des_speed = maxvel;// move with constant speed 
        stepup = 0;
      }
    else if( flag_stop == 0 && !stepup){
        des_speed = (100*des_speed-rate)/100; // decelerate
        if(des_speed<=0){
          flag_stop = 1;
          des_speed = 0;
          dir = -1*dir;
          stepup = 1;
          dist_stop = act_disty;
          stop_time = millis();
          ramptime = millis();
        }
      }
  }
  else{
    if(act_disty>(dist_low + accl_d) && flag_stop == 0){// for dir == -1
        if(stepup) accl_d = dist_stop - act_disty;
        des_speed = maxvel; // move with constant speed 
        stepup = 0;
      }
    else if(flag_stop == 0 && !stepup){
        des_speed = (100*des_speed-rate)/100; // decelerate
        if(des_speed<=0){
          flag_stop = 1;
          des_speed = 0;
          dir = -1*dir;
          stepup = 1;
          dist_stop = act_disty;
          stop_time = millis();
          ramptime = millis();
        }
      }
  }
  
//  Serial.print(act_disty);
//  Serial.print("\t");
//  Serial.print(accl_d);
//  Serial.print("\t");
  return abs(des_speed)*dir;
}
double vel_PID(double desired_vel){
  vel_error = desired_vel-velocity;
  d_vel_error = vel_error-prev_vel_error;/*if(!(abs(vel_error-prev_vel_error)<2)) */
//  else d_vel_error = 0;
  i_vel_error += vel_error;/*if(abs(vel_error)<1) */
//  else i_vel_error = 0;
  vel_ctrl = kp_v*vel_error+ki_v*i_vel_error+kd_v*d_vel_error;
  vel_ctrl = constrain(vel_ctrl, -100, 100);
//  Serial.print(" ki*e ");
//  Serial.print(ki_v*i_vel_error);
//  Serial.print("\to/p ");
//  Serial.print(vel_ctrl);
  prev_vel_error = vel_error;
  return vel_ctrl;
}
double vel_fuzzy(double desired_vel){
  vel_error = desired_vel-velocity;
  d_vel_error = vel_error-prev_vel_error;/*if(!(abs(vel_error-prev_vel_error)<3))*/
  vel_ctrl = -vel.centroid_correction( vel_error, d_vel_error);;
  vel_ctrl = constrain(vel_ctrl, -100, 100);
//  Serial.print(flag_stop);
//  Serial.print(ki_v*i_vel_error);
//  Serial.print("\t ");
//  Serial.print(vel_ctrl);
//  Serial.print("\t ");
  prev_vel_error = vel_error;
  return (vel_error<1)? vel_ctrl: 1.5*vel_ctrl;
}
