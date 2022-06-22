void xbox_control(){
//    Serial.print("c");
    Usb.Task();
    pwm_front_right = 127;
    pwm_front_left = 127;
    pwm_back_right = 127;
    pwm_back_left = 127;
    if(Xbox.getButtonClick(START)){
      read_lidar1();
      prev_distx = dist1;
      Serial.print(prev_distx);
      Serial.print("\t");
      Serial.println(dist1);
      flag_auto = !flag_auto;
      timer = millis();
    }
    if(Xbox.getButtonClick(BACK)){
      flag_slosh = 0;
      flag_auto = 0;
    }
    if(flag_auto && !flag_slosh){
      move_straight(0);
      dist_ctrl(100);
    }
    else if(!flag_slosh){
    if (Xbox.Xbox360Connected) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 ){
        move_left();
      }
      if( Xbox.getAnalogHat(LeftHatX) < -13500) {
        Serial.print("left");
        move_right();
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 ){
        move_front();
      }
      if( Xbox.getAnalogHat(RightHatY) < -7500) {
        move_back();
      }
      if( Xbox.getButtonPress(L2)>100) {
        rotate_clockwise();
      }
      if( Xbox.getButtonPress(R2)>100) {
        rotate_anticlockwise();
      }
      if (Xbox.getButtonClick(DOWN))
        if(k>5)
          k -= 5;
      if (Xbox.getButtonClick(UP))
        if (k<120)
          k += 5;
      if(Xbox.getButtonClick(START))
      {
        flag_auto = !flag_auto;
      }
    }
    }
//    print_pwm();
}

void move_front(){
  pwm_front_right += k;
  pwm_front_left += k;
  pwm_back_right += k;
  pwm_back_left += k;
}

void move_back(){
  pwm_front_right -= k;
  pwm_front_left -= k;
  pwm_back_right -= k;
  pwm_back_left -= k;
}

void move_left(){
  pwm_front_right -= k;
  pwm_front_left += k;
  pwm_back_right += k;
  pwm_back_left -= k;
}

void move_right(){
  pwm_front_right += k;
  pwm_front_left -= k;
  pwm_back_right -= k;
  pwm_back_left += k;
}
void rotate_clockwise(){
  pwm_front_right += k;
  pwm_front_left -= k;
  pwm_back_right += k;
  pwm_back_left -= k;
}
void rotate_anticlockwise(){
  pwm_front_right -= k;
  pwm_front_left += k;
  pwm_back_right -= k;
  pwm_back_left += k;
}
