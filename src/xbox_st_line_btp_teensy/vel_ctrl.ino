void vel_calc(){
  static double temp;
  temp = double(act_disty-prev_ac_disty)/(double(millis()-timer)/10);
//  Serial.print('\t');
//  Serial.println(millis()-timer);
//  Serial.println('\t');
  timer = millis();
  prev_ac_disty = act_disty;
  Serial.print(temp);
  Serial.print('\t');
  filterVel.add(temp);
  velocity = filterVel.getMedian();
}
