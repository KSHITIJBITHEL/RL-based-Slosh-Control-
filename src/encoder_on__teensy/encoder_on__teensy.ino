/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 
 #define outputA 34
 #define outputB 35
 int counter = 0;  
 void setup() { 
   pinMode (outputA,INPUT);
   pinMode (outputB,INPUT);
   
   Serial.begin (9600);
   // Reads the initial state of the outputA
   attachInterrupt(digitalPinToInterrupt(outputA), ISR, RISING);
 } 
 void loop() { 
  Serial.println(counter);
  delay(50);
 }
