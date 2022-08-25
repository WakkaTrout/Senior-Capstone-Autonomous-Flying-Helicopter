#include "global_variables.h"

#include <float.h>


inline void SetDesiredPosition(float yaw, float pitch, float &desiredyaw, float &desiredpitch)
{
  //From the position data, plan route given the desire mode on the reciever_m_pin
  // check desired mode from reciever_m_pin
  if(mode == 1)
  {
    //switch statement to model route planning state machine
    switch(flight_state_1)
    {
      //Interpolation state
      case 0:    
        //check to see if accomplished position to go to next position
        if(abs(desiredpitch-pitch) < 2*FLT_EPSILON) //I don't know how good this is for checking if we have arrived or not, might consider changing to a bigger margin
        {
              //if achieved segment (meaning got to the destination and not an interpolated position), then go to next segment
              if(desiredpitch == m1_pitch_route[m1_curr_state])
              {
                //if don't want to hold for any time, immediately go to next interpolation/power down
                if(m1_timer_route[m1_curr_state] == 0)
                {
                  ++m1_curr_state;
                  //if no more route left, go to power down state
                  if(m1_curr_state > m1_num)
                  {
                    flight_state_1 = 2;
                    powerDown = true;
                  }
                  //start next interpolation
                  else
                  {
                    float deltap =  m1_pitch_route[m1_curr_state] - desiredpitch;
                    float adeltap = abs(deltap);
            
                    //add/subtract c degrees to the current desired position to get closer to the goal
                    if(adeltap > cdeg)
                    {
                      desiredpitch += cdeg*deltap/adeltap;
                    }
                    else
                    {
                      desiredpitch = m1_pitch_route[m1_curr_state];
                    }
                  }
                }
                //we wish to hold our position for some time, so go to hold state
                else
                {
                  flight_state_1 = 1;
                  timerStart = micros();
                }
              }
              //if not at end of segment, interpolate
              else
              {
                float deltap =  m1_pitch_route[m1_curr_state] - desiredpitch;
                float adeltap = abs(deltap);
        
                //add/subtract c degrees to the current desired position to get closer to the goal
                if(adeltap > cdeg)
                {
                  desiredpitch += cdeg*deltap/adeltap;
                }
                else
                {
                  desiredpitch = m1_pitch_route[m1_curr_state];
                }
              }
          }
          break;
          
      //hold state
      case 1:
        //if timer expired, go to interpolation state
        if(micros() - timerStart >= m1_timer_route[m1_curr_state])
        {
          //then move onto next segment
          ++m1_curr_state;
          
          //if no more route left, go to power down state
          if(m1_curr_state > m1_num)
          {
            flight_state_1 = 2;
            powerDown = true;
          }
          //go back to interpolation state if more route left
          else
          {
            flight_state_1 = 0;
            
            //begin interpolation step
            float deltap =  m1_pitch_route[m1_curr_state] - desiredpitch;
            float adeltap = abs(deltap);
        
            //add/subtract c degrees to the current desired position to get closer to the goal
            if(adeltap > cdeg)
            {
              desiredpitch += cdeg*deltap/adeltap;
            }
            else
            {
              desiredpitch = m1_pitch_route[m1_curr_state];
            }
          }
        }
        break;
        
      //power down state
      case 2:
        //TO DO
        break;
    }    
  }    
  else if(mode == 2)
  {
    //switch statement to model route planning state machine
    switch(flight_state_2)
    {
      //Interpolation state
      case 0:    
        //check to see if accomplished position to go to next position
        if(abs(desiredyaw-yaw) < 2*FLT_EPSILON && abs(desiredpitch-pitch) < 2*FLT_EPSILON) //I don't know how good this is for checking if we have arrived or not, might consider changing to a bigger margin
        {
              //if achieved segment (meaning got to the destination and not an interpolated position), then go to next segment
              if(desiredyaw == m2_yaw_route[m2_curr_state] && desiredpitch == m2_pitch_route[m2_curr_state])
              {
                //if don't want to hold for any time, immediately go to next interpolation/power down
                if(m2_timer_route[m2_curr_state] == 0)
                {
                  ++m2_curr_state;
                  //if no more route left, go to power down state
                  if(m2_curr_state > m2_num)
                  {
                    flight_state_2 = 2;
                    powerDown = true;
                  }
                  //more route, so start next interpolation
                  else
                  {
                    float deltay = m2_yaw_route[m2_curr_state]-desiredyaw;
                    float deltap =  m2_pitch_route[m2_curr_state] - desiredpitch;
                    float adeltay = abs(deltay);
                    float adeltap = abs(deltap);
            
                    //add/subtract c degrees to the current desired position to get closer to the goal
                    if(adeltay > cdeg)
                    {
                      desiredyaw += cdeg*deltay/adeltay;
                    }
                    else
                    {
                      desiredyaw = m2_yaw_route[m2_curr_state];
                    }
                    if(adeltap > cdeg)
                    {
                      desiredpitch += cdeg*deltap/adeltap;
                    }
                    else
                    {
                      desiredpitch = m2_pitch_route[m2_curr_state];
                    }
                  }
                }
                //we wish to hold our position for some time, so go to hold state
                else
                {
                  flight_state_2 = 1;
                  timerStart = micros();
                }
              }
              //if not at end of segment, interpolate
              else
              {
                float deltay = m2_yaw_route[m2_curr_state]-desiredyaw;
                float deltap =  m2_pitch_route[m2_curr_state] - desiredpitch;
                float adeltay = abs(deltay);
                float adeltap = abs(deltap);

                //TO DO: Make two cdeg's, independent for yaw and pitch and make them a function of the direction needed to go
                //Consider moving the calc to the when the state starts

                //add/subtract c degrees to the current desired position to get closer to the goal
                if(adeltay > cdeg)
                {
                  desiredyaw += cdeg*deltay/adeltay;
                }
                else
                {
                  desiredyaw = m2_yaw_route[m2_curr_state];
                }
                if(adeltap > cdeg)
                {
                  desiredpitch += cdeg*deltap/adeltap;
                }
                else
                {
                  desiredpitch = m2_pitch_route[m2_curr_state];
                }
              }
          }
          break;
          
      //hold state
      case 1:
        //if timer expired, go to interpolation state
        if(micros() - timerStart >= m2_timer_route[m2_curr_state])
        {
          //then move onto next segment
          ++m2_curr_state;
          
          //if no more route left, go to power down state
          if(m2_curr_state > m2_num)
          {
            flight_state_2 = 2;
            powerDown = true;
          }
          //go back to interpolation state if more route left
          else
          {
            flight_state_2 = 0;

            //Start the first interpolation here
            float deltay = m2_yaw_route[m2_curr_state]-desiredyaw;
            float deltap =  m2_pitch_route[m2_curr_state] - desiredpitch;
            float adeltay = abs(deltay);
            float adeltap = abs(deltap);

            //TO DO: Make two cdeg's, independent for yaw and pitch and make them a function of the direction needed to go
            //Consider moving the calc to when the state starts and storing and reusing for each extra interpolation step

            //add/subtract c degrees to the current desired position to get closer to the goal
            if(adeltay > cdeg)
            {
              desiredyaw += cdeg*deltay/adeltay;
            }
            else
            {
              desiredyaw = m2_yaw_route[m2_curr_state];
            }
            if(adeltap > cdeg)
            {
              desiredpitch += cdeg*deltap/adeltap;
            }
            else
            {
              desiredpitch = m2_pitch_route[m2_curr_state];
            }
          }
        }
        break;
        
      //power down state
      case 2:
        //TO DO
        break;
    } 
  }
}
