#ifndef _GLOBAL_VARS_H_
#define _GLOBAL_VARS_H_

#include <Servo.h>
#include <Adafruit_BNO08x.h>

int manual_state;
int state_count = 0;
int potential_state;
Servo servo;
bool powerDown = false;
bool powerUp = true;
bool hasItem = false;
long timeStartUp = 0

//These variables keep track of the pulse width coming from the state transition button on the controller
volatile unsigned long pulseInTimeBegin = micros();
volatile unsigned long pulseInTimeEnd = micros();
unsigned long button_pressed = 950;
volatile bool newPulseDurationAvailable = false;

sh2_SensorValue_t sensorValue;
Adafruit_BNO08x  bno08x(BNO08X_RESET);

float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

float integral_sumpitch = 0.0;
float integral_sumyaw = 0.0;
float integral_sumpitch2 = 0.0;
float integral_sumyaw2 = 0.0;
float prevpitch = 0.0;
float prevroll = 0.0;
float prevyaw = 0.0;

//current power values in the "x" (yaw) and "y" (pitch) directions
float ux = 0.0;
float uy = 0.0;

//Steady state power of the helicopter
//const float c = 183;

//should be initialized on startup
float desiredyaw;//x - limit to range -180 to 180 degrees
float desiredpitch;//y -limit to 45 degrees (+- 15 degrees)

//State of the actuators
float servoAngle = 90.0;
float motorStrength = 0.0;

// Tube length in meters 
#define tube_length 0.5

//Mode of the helicopter
int mode = 1;

//State of the route controller
int flight_state_1 = 0;
int flight_state_2 = 0;
unsigned long timerStart;
unsigned long timerEnd;

//Interpolation constant
const float cdeg = 1.0;

//Route data for simple takeoff then landing
#define m1_num 2
float m1_pitch_route[m1_num] = {25.0, 0.0};
unsigned long m1_timer_route[m1_num] = {500, 10000};
int m1_curr_state = 0;

//Route data for simple takeoff, trip around half sphere, then landing
#define m2_num 6
float m2_yaw_route[m2_num] = {0.0, 90.0, 180.0, -90.0, 0.0, 0.0};
float m2_pitch_route[m2_num] = {25.0, 25.0, 25.0, 25.0, 25.0, 0.0};
unsigned long m2_timer_route[m2_num] = {500, 500, 500, 500, 500, 500};
int m2_curr_state = 0;

#endif
