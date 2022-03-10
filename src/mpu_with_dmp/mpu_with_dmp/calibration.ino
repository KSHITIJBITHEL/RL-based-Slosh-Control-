//--- Function To Calibrate the yaw pitch roll offsets ---//
void calibrate (){
    static float sum_yaw = 0, sum_pitch = 0, sum_roll = 0;
//  if programming failed, don't try to do anything
    if (!dmpReady) return;

//  Wait for the readings to saturate
    for (int i =0; i<=800; i++){
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) ;
    got_intr();
    Serial.println(fifoCount);
    }
    Serial.println(" Starting Calibration ");
    for (int i =0; i<10; i++){
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) ;
    got_intr();
    sum_yaw = sum_yaw + ypr[0]; 
    sum_pitch = sum_pitch + ypr[1]; 
    sum_roll = sum_roll + ypr[2]; 
    }
    yaw_offset = sum_yaw/10.0;
    pitch_offset = sum_pitch/10.0;
    roll_offset = sum_roll/10.0;
    print_offsets();
}
