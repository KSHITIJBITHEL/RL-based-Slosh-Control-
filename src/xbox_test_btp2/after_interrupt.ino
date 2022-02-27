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

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            print_quat();
        #endif

        #ifdef OUTPUT_COMPASS
            // вывод данных магнитометра
            mpu.dmpGetMag(mag, fifoBuffer);                     // get raw data from DMP
            f_mag[0] = mag[1]*((asax-128)*0.5/128+1);           // transform and change the orientations of the X, Y, Z axes
            f_mag[1] = mag[0]*((asay-128)*0.5/128+1);           // according to the specification
            f_mag[2] = -mag[2]*((asax-128)*0.5/128+1);          // to the sensor page 38
            VectorFloat v_mag(f_mag[0], f_mag[1], f_mag[2]);    // create a magnetometer vector
            v_mag = v_mag.getNormalized();                      // normalize the vector
            v_mag = v_mag.getRotated(&q_mag);                   // rotate
            float phi = atan2(v_mag.y, v_mag.x)/3.1416;         // get the angle values ​​in radians between X, Y
            Quaternion q_mag(0.1*phi, 0, 0, 1);                 // create a corrective quaternion
            q = q_mag.getProduct(q);                            // multiply the quaternions to correct the main
            q.normalize();                                      // normalize the quaternion
            print_mag();
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
   
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
