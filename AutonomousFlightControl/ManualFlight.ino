//This function manages manual flight. It reads in the values sent by the RC controller and actuates the servo and motor accordingly
inline void manual_flight()
{
  //set servo angle
  
  long pulsewidthmicro = pulseIn(reciever_servo_pin, HIGH);
  //go if not timed out
  if(pulsewidthmicro > 0)
  {
    pulsewidthmicro = map(pulsewidthmicro-175, 989, 2041, 500, 2450);
    digitalWrite(servo_pin, HIGH);
    delayMicroseconds(pulsewidthmicro);
    digitalWrite(servo_pin, LOW);
    
    //set motor strength
    long pulsewidthmicromot = pulseIn(reciever_throt_pin, HIGH);
    //go if not timed out
    if(pulsewidthmicromot > 0)
    {
      servo.writeMicroseconds(3020-pulsewidthmicromot);
    }
  }
}


  /*
  Serial.print(" Motor Strength");
  Serial.println(pulsewidthmicro);
  */
