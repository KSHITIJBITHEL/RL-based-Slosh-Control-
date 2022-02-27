void read_lidar1(){
   if (Serial4.available()) { //check if serial port has data input
  
  if(Serial4.read() == HEADER) { //assess data package frame header 0x59 
    uart1[0]=HEADER;
    if (Serial4.read() == HEADER) { //assess data package frame header 0x59
    uart1[1] = HEADER;
    for (int i = 2; i < 9; i++) { //save data in array
    uart1[i] = Serial4.read(); 
    }
//    Serial.println("here1");
    check = uart1[0] + uart1[1] + uart1[2] + uart1[3] + uart1[4] + uart1[5] + uart1[6] + uart1[7]; 
    if (uart1[8] == (check & 0xff)){ //verify the received data as per protocol
      dist1 = uart1[2] + uart1[3] * 256; //calculate distance value 
      strength1 = uart1[4] + uart1[5] * 256; //calculate signal strength value 
      temprature1 = uart1[6] + uart1[7] *256;//calculate chip temprature 
      temprature1 = temprature1/8 - 256;
//      Serial.print("dist1 = ");
//      Serial.print(dist1); //output measure distance value of LiDAR Serial.print('\t');
  } }
  } }
  Serial4.clear();
}
void read_lidar2(){
   if (Serial3.available()) { //check if serial port has data input
  
  if(Serial3.read() == HEADER) { //assess data package frame header 0x59 
    uart2[0]=HEADER;
    if (Serial3.read() == HEADER) { //assess data package frame header 0x59
    uart2[1] = HEADER;
    for (int i = 2; i < 9; i++) { //save data in array
    uart2[i] = Serial3.read(); 
    }
//    Serial.println("here2");
    check = uart2[0] + uart2[1] + uart2[2] + uart2[3] + uart2[4] + uart2[5] + uart2[6] + uart2[7]; 
    if (uart2[8] == (check & 0xff)){ //verify the received data as per protocol
      dist2 = uart2[2] + uart2[3] * 256; //calculate distance value 
      strength2 = uart2[4] + uart2[5] * 256; //calculate signal strength value 
      temprature2 = uart2[6] + uart2[7] *256;//calculate chip temprature 
      temprature2 = temprature2/8 - 256;
//      Serial.print("dist2 = ");
//      Serial.print(dist2); //output measure distance value of LiDAR Serial.print('\t');

  } }
  } }
  Serial3.clear();
}
