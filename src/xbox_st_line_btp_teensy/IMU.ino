// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
  mpuInterrupt = true;
}
// ================================================================
// ===     Function To Calibrate the yaw pitch roll offsets     ===
// ================================================================
void calibrate (){
    static float sum_yaw = 0, sum_pitch = 0, sum_roll = 0;
//  if programming failed, don't try to do anything
    if (!dmpReady) return;

//  Wait for the readings to saturate
    for (int i =0; i<=800; i++){
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) ;
    got_intr();
//    Serial.println(fifoCount);
    }
//    Serial.println(" Starting Calibration ");
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
    print_gyro_offsets();
}
// ================================================================
// ===       Function To execute after getting interrupt        ===
// ================================================================
void got_intr(){
  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
    }
}
