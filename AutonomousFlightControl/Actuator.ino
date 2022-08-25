//This function sets up the servo object as well as puts the throttle to minimum as that is necessary before flight can start
inline void initializeMotor(int pin)
{
   servo.attach(motor_pin, 950, 2050);
   delay(500);
   servo.writeMicroseconds(977);
   delay(1000);
   servo.writeMicroseconds(1500);
   delay(1500);
}

//This function sets the motor's strength to the given percentage of the maximum strength
inline void setMotor(int pin, float percent)
{
  long pulsewidthmicro;

  //since max is 2040 us and min is 1500 us, then range is 540 us, so get the percent of the range and add that to the min to get val
  pulsewidthmicro = (long)((percent*540.0)/100.0 + 1500.0);
  
  //if greater than max, cap it at max (2040 us)
  //Actually, it is set to 83.3% (1950 us) just so it does not go to fast
  if(pulsewidthmicro > 1950){
    pulsewidthmicro = 1950;
  }
  //if less than min, cap it at min
  else if(pulsewidthmicro < 1500)
  {
    pulsewidthmicro = 1500;
  }
  servo.writeMicroseconds(pulsewidthmicro);
}

//sets the servo to a given angle
inline void setServoAngle(int pin,float angle)
{
    long pulsewidthmicro;
    //limit the angle of the servo
    if(angle > 95.0)
    {
        angle = 95.0;
    }
    else if(angle < 85.0)
    {
        angle = 85.0;
    }
    //map angle to delay amount (empirical formula derived using excel)
    pulsewidthmicro = (long)((angle*65.0)/9.0+475.0);
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulsewidthmicro);
    digitalWrite(pin, LOW);
  
}


/*
void servoTest(int pin)
{
  for(int i = 1300; i <3000; ++i)
  {
    unsigned long delay_time;
    delay_time = micros();
    digitalWrite(servo_pin, HIGH);
    delayMicroseconds(i);
    digitalWrite(servo_pin, LOW);
    delay_time = micros()-delay_time;
    Serial.print("Elapsed time: ");
    Serial.print(delay_time);
    Serial.print("\n");
    delay(100);
  }
}*/
