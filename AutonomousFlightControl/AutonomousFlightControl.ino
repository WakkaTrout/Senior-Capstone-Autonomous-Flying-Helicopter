//Pins for Motion Sensor
//Declare CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

#define BNO08X_RESET -1

//Pin for Servo
#define servo_pin 10
#define motor_pin 11
#define reciever_pin 13
#define reciever_throt_pin 9
#define reciever_servo_pin 12
#define reciever_m_pin 5

#include <math.h>
#include "global_variables.h"



//This function setups the connection to the position sensor, the input/output of the pins, the intial servo angle, and starts up the motor
void setup(void) {
  delay(1600);
  // Try to initialize the position sensor!
  if (!bno08x.begin_I2C()) {
   // Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }

  //Tell the position sensor that we wish to recieve orientation values
 // Serial.println("BNO08x Found!");
  setReports();

  //setup the pins for input and output
  pinMode(reciever_pin, INPUT);
  pinMode(reciever_servo_pin, INPUT);
  pinMode(reciever_throt_pin, INPUT);
  pinMode(servo_pin, OUTPUT);

  //set the initial angle
  setServoAngle(servo_pin, 90.0);
  
  //intialize the motor
  initializeMotor(motor_pin);
  
  //set the intial state to automatic
  manual_state = 0;
  potential_state = 0;

  //attach the mode switching button interrupt
  attachInterrupt(digitalPinToInterrupt(reciever_pin), state_interrupt, CHANGE);

  //initialize the flight path to the starting position of the helicopter
  while(! bno08x.getSensorEvent(&sensorValue)) {}
  GetPositionFromQuaternion(pitch, roll, yaw, sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i, sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
  desiredyaw = yaw;
  desiredpitch = pitch;


  //TO DO:
  //Loop to power up the motor so it is a bit more gradual
  //Tune this loop to fit startup better. Also, move this into automatic flight somehow
  delay(100);
  uy = 15.0;
  setMotor(motor_pin, uy);
  timeStartUp = millis();
}



// Here is where you define the sensor outputs you want to receive
void setReports(void) {
 // Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
 //   Serial.println("Could not enable game vector");
  }
}


//This function continuously loops, checking for state change between manual and automatic flight and then executes the correct action
void loop() 
{
   //read button on rc controller to see if we should transition between manual and automatic flight
   if(newPulseDurationAvailable)
   {
      newPulseDurationAvailable = false;
      unsigned long temp = pulseInTimeEnd-pulseInTimeBegin;
      //replace with an if statement that needs to meet a count of 2 or 3 before it actually changes the button_pressed value to account for glitches
      if(temp < 2500 && temp > 400)
      {
        //if within range of transition state, potentially transition or increase count
        if(temp < potential_state + 20 && temp > potential_state -20)
        {
          //if exceeded count, transition to next state
          if(state_count == 1)
          {
            button_pressed = temp;
            state_count = 0;
          }
          //else, increase count
          else
          {
            ++state_count;
          }
        }
        //if not in range, reset count and set the transition state to the temp value
        else
        {
          state_count = 0;
          potential_state = temp;
        }
      }
   }
 //  Serial.print("State ");
  // Serial.print(manual_state);
  // Serial.print(" Button Pressed value: ");
  // Serial.println(button_pressed);

   //this switch statement runs the appropriate states code and transitions to another state given the value read from the button on the rc controller
   switch(manual_state)
   {
      case 0: //automatic flight state button pressed state
              automatic_flight();
              //check to see if button unpressed to transition to case 1
              if(button_pressed < 1450) //neutral is 1500 (above 1500 is button pressed and below is button unpressed) but there is some error (give or take 50), so state transition should past the threshold amount 
              {
                manual_state = 1;
              }
              break;
      case 1: //automatic flight state button unpressed state
              automatic_flight();
              //check to see if button is pressed to transition to case 2
              if(button_pressed > 1550)
              {
                manual_state = 2;
              }
              break;
      case 2: //manual flight state button pressed state
              manual_flight();
              //check to see if button is unpressed to transition to case 3
              if(button_pressed < 1450)
              {
                manual_state = 3;
              }
              break;
      case 3: //manual flight state button unpressed state
              manual_flight();
              //check to see if button is pressed to tranisiton to case 0;
              if(button_pressed > 1550)
              {
                manual_state = 0;
              }
              break;
              //if somehow get out of these states, then default to manual flight
      default: manual_flight();
               break;
   }
}

//this function is an interrupt handler to measure the pulse from the reciever for the manual/automatic button
void state_interrupt()
{
  if(digitalRead(reciever_pin) == HIGH){
    pulseInTimeBegin = micros();
  }
  else
  {
    pulseInTimeEnd = micros();
    newPulseDurationAvailable = true;
  }
}

//These are used to calculate the widths of the delays for each channel, but it is not needed at the moment
void receiever_serv_interrupt()
{
  
}

void receiever_throt_interrupt()
{
  
}

void reciever_m_interrupt()
{
  
}
