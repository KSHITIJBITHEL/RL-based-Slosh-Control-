//--- Prints Capacitance Values ---//
//void cap_print(){
//  Serial.print(" C1 : ");
//  Serial.print(cap1);
//  Serial.print(" C2 : ");
//  Serial.print(cap2);
//  Serial.print(" C3 : ");
//  Serial.print(cap3);
//  Serial.print(" C4 : ");
//  Serial.print(cap4);
//}

//--- Plots Capacitance Values ---//
void cap_plot(){

//  Serial.println(x);
  Serial.println(cap1*5);
//  Serial.print('\t');
//  Serial.print(cap2);
//  Serial.print("\t");
//  Serial.println(cap3);
//  Serial.print("\t");
//  Serial.println(cap4);
//  Serial.print("\t");
}
void print_offset(){
  Serial.print(cap1_offset);
  Serial.print(" ");
  Serial.print(cap2_offset);
  Serial.print(" ");
  Serial.print(cap3_offset);
  Serial.print(" ");
  Serial.println(cap4_offset);
}
