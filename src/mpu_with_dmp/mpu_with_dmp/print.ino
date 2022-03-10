void print_reading(){
        Serial.print("ypr\t");
        Serial.print(yaw_reading);
        Serial.print("\t");
        Serial.print(pitch_reading);
        Serial.print("\t");
        Serial.println(roll_reading);
}
void print_ypr(){
        Serial.print("ypr\t");
        Serial.print(ypr[0]);
        Serial.print("\t");
        Serial.print(ypr[1]);
        Serial.print("\t");
        Serial.println(ypr[2]);
}
void print_offsets(){
    Serial.print("Offsets : ");
    Serial.print(yaw_offset);
    Serial.print(" ");
    Serial.print(pitch_offset);
    Serial.print(" ");
    Serial.println(roll_offset);
}
void print_mag(){
      Serial.print(q.w);
      Serial.print(",");
      Serial.print(q.x);
      Serial.print(",");
      Serial.print(q.y);
      Serial.print(",");
      Serial.println(q.z);
}

void print_quat(){
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
}
