void ISR()
{
  if(digitalRead(outputB)== HIGH) counter++;
  else counter--;
}
