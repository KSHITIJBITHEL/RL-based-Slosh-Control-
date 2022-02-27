void xbox_control(float pid){
    Usb.Task();
    pwm_front_right = 127+pid;
    pwm_front_left = 127-pid;
    pwm_back_right = 127+pid;
    pwm_back_left = 127-pid;
    if (Xbox.Xbox360Connected) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 ){
        move_left();
        if(debug == 1) Serial.print("right");
      }
      if( Xbox.getAnalogHat(LeftHatX) < -7500) {
        if(debug == 1) Serial.print("left");
        move_right();
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 ){
        move_front();
        if(debug == 1) Serial.print("front");
      }
      if( Xbox.getAnalogHat(RightHatY) < -7500) {
        move_back();
        if(debug == 1) Serial.print("back");
      }
      if( Xbox.getButtonPress(L2)>100) {
        rotate_clockwise();
        if(debug == 1) Serial.print("clockwise");
      }
      if( Xbox.getButtonPress(R2)>100) {
        rotate_anticlockwise();
        if(debug == 1) Serial.print("anti clockwise");
      }
      if (Xbox.getButtonClick(DOWN))
        if(k>5)
          k -= 5;
      if (Xbox.getButtonClick(UP))
        if (k<120)
          k += 5;
    }
    print_pwm();
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
