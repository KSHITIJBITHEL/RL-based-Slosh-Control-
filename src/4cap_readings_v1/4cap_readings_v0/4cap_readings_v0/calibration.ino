void calibrate (){
    static float sum1 = 0,sum2 = 0,sum3 = 0,sum4 = 0;
    for (int i = 0; i<100;i++){
    cap1 = read_cap(0,capdac1,value1);
    cap2 = read_cap(1,capdac2,value2);
    cap3 = read_cap(2,capdac3,value3);
    cap4 = read_cap(3,capdac4,value4); 
    sum1 += cap1;
    sum2 += cap2;
    sum3 += cap3;
    sum4 += cap4;
    }
    cap1_offset = sum1/100.0;
    cap2_offset = sum2/100.0;
    cap3_offset = sum3/100.0;
    cap4_offset = sum4/100.0;
}
