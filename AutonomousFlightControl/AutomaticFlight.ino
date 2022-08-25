#include "global_variables.h"

//This function manages automatic flight. It reads the current position, calculates where it should be, calculates how to get there, and then actuates to go there
void automatic_flight()
{

  if (bno08x.wasReset()) {
   // Serial.print("sensor was reset ");
    setReports();
  }

  if(powerUp)
  {
    //power up to 55 percent
    if(uy < 55.0)
    {
      //delay for x milliseconds before next increment
      if((millis()-timeStartUp) > 150)
      {
        timeStartUp = millis();
        uy += 0.5;
        setMotor(motor_pin, uy);
      }
    }
    //once at 50%, let the PID controller take over
    else
    {
      powerUp = false;
    }
  }
  else if(powerDown)
  {
    //TO DO (IF NEEDED): POWER DOWN GRADUALLY INSTEAD OF SETTING THE MOTOR TO EXACTLY 0
    //float servoAngle = 90.0;
    //float motorStrength = 0.0;
  }
  else
  {
    if (! bno08x.getSensorEvent(&sensorValue)) {
      return;
    }
    
    switch (sensorValue.sensorId) {
  
      //if the value read from the position sensor is a recieved position
      case SH2_GAME_ROTATION_VECTOR:
  
        //Get the yaw, pitch, and roll in degrees from the quaternion that comes from the positions sensor
        GetPositionFromQuaternion(pitch, roll, yaw, sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
  
        //Calculate the route given the current position
        SetDesiredPosition(yaw, pitch, desiredyaw, desiredpitch);

        //From route data, calculate angle and strength of motor using controller
        if(hasItem)
        {
          PIDcontrol2(yaw, pitch, desiredyaw, desiredpitch, servoAngle, motorStrength);
        }
        else
        {
          PIDcontrol(yaw, pitch, desiredyaw, desiredpitch, servoAngle, motorStrength);
        }
  
        //Actuate the servo and motor to achieve the position
        setServoAngle(servo_pin, servoAngle);
        setMotor(motor_pin, motorStrength);
        break;
    }
  }
}
