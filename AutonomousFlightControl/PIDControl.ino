#include "global_variables.h"

inline void PIDcontrol(float yaw, float pitch, float desiredyaw, float desiredpitch, float &servoAngle, float &motorStrength){ 
      //calculation of the error
      float eyaw = desiredyaw - yaw;
      float epitch = desiredpitch-pitch;

      //keep the error within the range -180 to 180 degrees going around the circle (don't need it for roll as that should be limited between -45 to 45)
      if(eyaw > 180)
      {
        eyaw -= 360;
      }
      else if(eyaw < -180)
      {
        eyaw += 360;
      }

      //calculations for the derivative (finite difference approximation) of the error
      float derivpitch = pitch-prevpitch;
      float derivyaw = yaw-prevyaw;

      prevpitch = pitch;
      prevyaw = yaw;

      //calculations for the integral of the error (approximate the integral with a summation)
      integral_sumpitch += epitch;// need to limit these values
      integral_sumyaw += eyaw; //need to limit this value

      //Limit the integral to not allow it to go above 200
      if(integral_sumpitch > 200)
      {
        integral_sumpitch = 200;
      }
      if(integral_sumpitch < -200)
      {
        integral_sumpitch = -200;
      }
      if(integral_sumyaw > 200)
      {
        integral_sumyaw = 200;
      }
      if(integral_sumyaw < -200)
      {
        integral_sumyaw = -200;
      }
     
       //dt =  0.0100235 sec
       //Ku-y = 0.0001550
       //Tu-y = about 3.90 sec

      //PID controller constants for the roll ("y" axis)
      const float kpy = 0.0001700;
      const float kdy = 0.0;
      const float kiy = 0.0;

      //PID controller constants for the yaw ("x" axis)
      const float kpx = 0.012500;
      const float kdx = 0.0;
      const float kix = 0.0;
      
      //PID controller equations
      ux = kpx*eyaw-kdx*derivyaw+kix*integral_sumyaw;
      uy += kpy*epitch-kdy*derivpitch+kiy*integral_sumpitch;

      //uy should not be negative as that is saying the helicopter should go into the ground
      if(uy < 0.0)
      {
        uy = 0.0;
      }
      
      //compute the motor strength and servo angle from the PID controller outputs
      servoAngle = atan2(uy, ux)*180.0/M_PI;

      //If statement potentially not needed anymore
      //If we are going downwards, need to reflect the motor over 90 degrees to get desired orientation
      if(servoAngle < 0.0)
      {
        servoAngle *= -1.0;
      }
      motorStrength = sqrt(0.000008*ux*ux+uy*uy);
}



//This PID controller is used for mode 3 when an item is being carried
//Should be changed to an adapative controller, but for now its fine
inline void PIDcontrol2(float yaw, float pitch, float desiredyaw, float desiredpitch, float &servoAngle, float &motorStrength){ 
      //calculation of the error
      float eyaw = desiredyaw - yaw;
      float epitch = desiredpitch-pitch;

      //keep the error within the range -180 to 180 degrees going around the circle (don't need it for roll as that should be limited between -45 to 45)
      if(eyaw > 180)
      {
        eyaw -= 360;
      }
      else if(eyaw < -180)
      {
        eyaw += 360;
      }

      //calculations for the derivative (finite difference approximation) of the error
      float derivpitch = pitch-prevpitch;
      float derivyaw = yaw-prevyaw;

      prevpitch = pitch;
      prevyaw = yaw;

      //calculations for the integral of the error (approximate the integral with a summation)
      integral_sumpitch2 += epitch;// need to limit these values
      integral_sumyaw2 += eyaw; //need to limit this value

      //Limit the integral to not allow it to go above 100
      if(integral_sumpitch2 > 100)
      {
        integral_sumpitch2 = 100;
      }
      if(integral_sumpitch2 < -100)
      {
        integral_sumpitch2 = -100;
      }
      if(integral_sumyaw2 > 100)
      {
        integral_sumyaw2 = 100;
      }
      if(integral_sumyaw2 < -100)
      {
        integral_sumyaw2 = -100;
      }
     

      //PID controller constants for the roll ("y" axis)
      const float kpy = 0.1;
      const float kdy = 0.1;
      const float kiy = 0.01;

      //PID controller constants for the yaw ("x" axis)
      const float kpx = 0.1;
      const float kdx = 0.1;
      const float kix = 0.01;

      //This constant models the amount of thrust needed to cancel out the weight of the helicopter (this should be the motor strength that cancels the weight of the whole helicopter)
     // const float c = 200;
      
      //PID controller equations
      ux += kpx*eyaw-kdx*derivyaw+kix*integral_sumyaw2;
      //uy = kpy*epitch-kdy*derivpitch+kiy*integral_sumpitch2+ c*sin((desiredpitch*M_PI)/180);
      uy += kpy*epitch-kdy*derivpitch+kiy*integral_sumpitch2;

      //uy should not be negative as that is saying the helicopter should go into the ground
      if(uy < 0.0)
      {
        uy = 0.0;
      }
      
      //compute the motor strength and servo angle from the PID controller outputs
      servoAngle = atan2(uy, ux)*180.0/M_PI;
      //If we are going downwards, need to reflect the motor over 90 degrees to get desired orientation
      if(servoAngle < 0.0)
      {
        servoAngle *= -1.0;
      }
      motorStrength = sqrt(ux*ux+uy*uy);

      /*
      Serial.print("yaw: ");
      Serial.print(yaw);
      Serial.print(" desired yaw: ");
      Serial.print(desiredyaw);
      Serial.print(" error yaw: ");
      Serial.print(eyaw);
      Serial.print(" yaw derivative: ");
      Serial.print(derivyaw);
      Serial.print(" yaw integral: ");
      Serial.println(integral_sumyaw);
      Serial.print("roll: ");
      Serial.print(roll);
      Serial.print(" desired roll: ");
      Serial.print(desiredroll);
      Serial.print(" error roll: ");
      Serial.print(eroll);
      Serial.print(" roll derivative: ");
      Serial.print(derivroll);
      Serial.print(" roll integral: ");
      Serial.println(integral_sumroll);
      Serial.print("ux: ");
      Serial.print(ux);
      Serial.print(" uy: ");
      Serial.print(uy);
      Serial.print(" Servo Angle: ");
      Serial.print(servoAngle);
      Serial.print(" Motor Strength");
      Serial.println(motorStrength);
      */
}
