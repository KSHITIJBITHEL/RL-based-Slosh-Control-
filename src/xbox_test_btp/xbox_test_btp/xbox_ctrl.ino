void xbox_control(){
    Usb.Task();
    pwm_front_right = 127;
    pwm_front_left = 127;
    pwm_back_right = 127;
    pwm_back_left = 127;
    if (Xbox.Xbox360Connected) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500 ){
        move_left();
        Serial.print("right");
      }
      if( Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial.print("left");
        move_right();
      }
      if (Xbox.getAnalogHat(RightHatY) > 7500 ){
        move_front();
//        pwm_front_right = 127+40;
        Serial.print("front");
      }
      if( Xbox.getAnalogHat(RightHatY) < -7500) {
        move_back();
        Serial.print("back");
      }
      if( Xbox.getButtonPress(L2)>100) {
        rotate_clockwise();
//        Serial.print("back");
      }
      if( Xbox.getButtonPress(R2)>100) {
        rotate_anticlockwise();
//        Serial.print("back");
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
