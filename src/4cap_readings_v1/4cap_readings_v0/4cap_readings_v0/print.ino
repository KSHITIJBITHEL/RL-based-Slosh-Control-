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
//  Serial.print(cap1);
//  Serial.print('\t');
<<<<<<< HEAD
<<<<<<< HEAD
  Serial.print(cap2);
  Serial.print("\t");
  Serial.println(cap3);
//  Serial.print(",");
=======
  Serial.println(cap2);
//  Serial.print("\t");
//  Serial.println(cap3);
=======
  Serial.print(cap2);
  Serial.print("\t");
  Serial.println(cap3);
>>>>>>> 177c78a5deece39474b22868a2898bc01be28cd0
//  Serial.print("\t");
>>>>>>> 5a135e4e679808e55565937780f5c9e456c50d55
//  Serial.println(cap4);
//  Serial.print("\t");
//  Serial.println(10*(cap4-cap1));
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
