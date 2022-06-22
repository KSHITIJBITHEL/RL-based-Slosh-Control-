double STC (double x,double xd, double v,double capa,double capb){
  static double zeta1,zeta2,zeta3,vl,wl,sigma,b1,c1=1,dia= 11.8,M = 2.7, ms=0.375 ,h1,h2;
  static double force;
  h1 = 5;// (capa+0.388)/0.5011;
  h2 = 5;//(capb+0.3505)/0.4743;
  zeta1 = x-xd;
  zeta2 = v;
  zeta3 = atan((h2-h1)/dia);
  phi_dot = ((zeta3-phi)*1000.0)/(double(millis())-t_phi_dot);
  t_phi_dot = double(millis());
  phi = zeta3;
  b1 = 1/(M-ms*cos(zeta3)*cos(zeta3));
  sigma = c1*zeta2+c2*zeta1;
  wl = (sigma-c2*zeta1)*c2/c1; // c2*zeta2
  if(!incr) vel_integral = 0;
  vel_integral += k2*(signum(sigma))*double(millis()-t_stc)/(1000);
  vl = -(k1*(signum(sigma))*sqrt(abs(sigma)))- vel_integral;
  force = (vl-wl)/(c1*b1);
  if(!incr) des_v = 0;
   des_v = des_v+ (force*(double(millis())-t_stc))/(10000*M);
   des_v = constrain(des_v, -6,6);
//  Serial.print(double(millis())-t_stc);
//  Serial.print('\t');
  t_stc = millis();
//  Serial.print(x);
//  Serial.print('\t');
//  Serial.print((zeta2));
//  Serial.print('\t');
//  Serial.print(zeta3);
//  Serial.print('\t');
  Serial.print(vel_integral);
//  Serial.print('\t');
//  Serial.print(sigma);
  Serial.print('\t');
  Serial.print(vl);
//  Serial.print('\t');
//  Serial.print(incr);
//  Serial.print('\t');
//  Serial.print(c2);
  Serial.print('\t');
  Serial.println(des_v);
  
  return des_v;
}

double signum ( double x){
  if(x>0) return 1;
  else if(x<0) return -1;
  else return 0;
}
