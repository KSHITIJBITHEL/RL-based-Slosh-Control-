double STC (double x,double xd, double v,double h1,double h2){
  static double zeta1,zeta2,zeta3,vl,wl,sigma,b1,c1=1,dia= ,M = , ms= ,des_v;
  static double c2 = 2.6441,k1 = 1.114,k2 = 0.1001,force,vel_integral =0;
  zeta1 = x-xd;
  zeta2 = vel;
  zeta3 = atan((h2-h2)/dia);
  b1 = 1/(M-ms*cos(zeta3)*cos(zeta3));
  sigma = c1*zeta2+c2*zeta1;
  wl = (sigma-c2*zeta1)*c2/c1;
  vl_integral += k2*signum(sigma);
  vl = -k1*signum(sigma)*sqrt(abs(sigma))- vl_integral;
  force = (vl-wl)/(c1*b1);
  des_v = des_v+ force*(t_stc-millis())/(1000*M);
  t_stc = millis();
  return des_v;
}
