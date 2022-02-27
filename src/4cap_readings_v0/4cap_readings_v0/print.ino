//--- Prints Capacitance Values ---//
void cap_print(){
  Serial.print(" C1 : ");
  Serial.print(cap1);
  Serial.print(" \tC2 : ");
  Serial.print(cap2);
  Serial.print("\tC3 : ");
  Serial.print(cap3);
  Serial.print(" \tC4 : ");
  Serial.println(cap4);
}

//--- Plots Capacitance Values ---//
void cap_plot(){
  x = filcap1-filcap4;
  y = filcap3-filcap2;
  static double abs_slosh;
  abs_slosh = sqrt(x*x+y*y);
  if(abs_slosh>1) Serial.println("exceeded limit");
  Serial.print(String(x)+","+String(y)+"    "+String(abs_slosh)+"\n");
  
//  Serial.println(x);
//  Serial.print(cap1);
//  Serial.print(" ");
//  Serial.print(cap2);
//  Serial.print(" ");
//  Serial.print(cap3);
//  Serial.print(" ");
//  Serial.println(cap4);
}

void level(){
  h = (cap2)/(cap4-cap3);
//  Serial.print(cap2);
//  Serial.print("\t");
//  Serial.print((cap4-cap3));
//  Serial.print("\t");
  filter5.add(h);

  Serial.println(filter5.getMedian());
}
