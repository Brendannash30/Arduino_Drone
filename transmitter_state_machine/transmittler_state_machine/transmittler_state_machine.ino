#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

/*A basic 4 channel transmitter using the nRF24L01 module.*/
/*Credit ELECTRONOOBS, State machine and edits by B Nash*/
 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define INT_BUTTON_2 2
#define INT_BUTTON_3 3
volatile bool left_button = 0;  
volatile bool right_button = 0; 
/*Create a unique pipe out.*/
const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver


RF24 radio(9, 10); // select  CSN  pin

//drone states
enum Dif_States
{
  not_yet_started,
  starting,
  pot_throttle,
  hover
};

struct State_Mach
{
  Dif_States droneState;
  //Button_States buttonState;
};

State_Mach state;

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};
MyData data;

void resetData() 
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.
    
  data.throttle = 0;
  data.yaw = 90;
  data.pitch = 90;
  data.roll = 90;

}
//////////////////////////////pushbutton ISRs/////////////////////////////////
void service_2()
{
  //debounce pushbutton for accurate count
  delay(500);
  if (digitalRead(INT_BUTTON_2) == LOW) {
  left_button ^= 1;
  }
}
void service_3()
{
  //debounce pushbutton for accurate count
  delay(500);
  if (digitalRead(INT_BUTTON_3) == LOW) {
  right_button ^= 1;
  }
}
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////state helpers///////////////////////////////////
void start_seq()
{
  data.throttle = 0;
  data.yaw      = 255;
  data.pitch    = mapJoystickVal_others( analogRead(A2), 0, 520, 1021, true );
  data.roll     = mapJoystickVal_others( analogRead(A3), 0, 510, 1021, true );
}
void free_control()
{
  data.throttle = MapJoystickVal_throt( analogRead(A0), 0, 520, 1021, false );
  data.yaw      = mapJoystickVal_others( analogRead(A1),  0, 515, 1021, true );
  data.pitch    = mapJoystickVal_others( analogRead(A2), 0, 520, 1021, true );
  data.roll     = mapJoystickVal_others( analogRead(A3), 0, 510, 1021, true );
}
void hover_now()
{
  data.throttle = 50;
  data.yaw      = mapJoystickVal_others( analogRead(A1),  0, 515, 1021, true );
  data.pitch    = mapJoystickVal_others( analogRead(A2), 0, 520, 1021, true );
  data.roll     = mapJoystickVal_others( analogRead(A3), 0, 510, 1021, true );
}
/////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  //Start everything up
  Serial.begin(9600);
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();
  
  
  state.droneState = not_yet_started;
  //set up interrupts
  pinMode(2, INPUT_PULLUP); //pin pulled up until left pushbutton pressed
  attachInterrupt(digitalPinToInterrupt(2), service_2, RISING); //initiate interrupts
  pinMode(3, INPUT_PULLUP); //pin pulled up until right pushbutton pressed
  attachInterrupt(digitalPinToInterrupt(3), service_3, FALLING); //initiate interrupts
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int MapJoystickVal_throt(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 90);
  else
    val = map(val, middle, upper, 90, 180);
  return ( reverse ? 180 - val : val );
}
int mapJoystickVal_others(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 38, 128);
  else
    val = map(val, middle, upper, 128, 218);
  return ( reverse ? 255 - val : val );
}
void drone_state_machine(State_Mach &current)
{
  
  switch (current.droneState) {
    case not_yet_started:
      free_control();
      if(left_button == 1)
      {
        left_button = 0;
        right_button = 0;
        current.droneState = starting;
      }
      break;

    case starting:
      //coming into this state, left button = 1 and right button = 0
      start_seq();
      //click right, go to potentiometer throttle
      if(left_button == 1)
      {
        left_button = 0;
        right_button = 0;
        current.droneState = pot_throttle;
      }
      else if(right_button == 1)
      {
        left_button = 0;
        right_button = 0;
        current.droneState = hover;
      }
      
      break;

    case pot_throttle:
    //coming into this state, left button = 0 and right button = 0
      free_control();
      if(right_button == 1)
      {
        left_button = 0;
        right_button = 0;
        current.droneState = hover;
      }
      break;

    case hover:
    //coming into this state, left button = 0 and right button = 0
      hover_now();
      if(left_button == 1)
      {
        left_button = 0;
        right_button = 0;
        current.droneState = pot_throttle;
      }
      break;

    // default:
    //   // Handle unknown state (optional)
    //   break;
  }
}
void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks till they send the correct values.
  // data.throttle = MapJoystickVal_others( analogRead(A0), 13, 520, 1015, false );
  // data.yaw      = MapJoystickVal_others( analogRead(A1),  1, 512, 1020, true );
  // data.pitch    = MapJoystickVal_others( analogRead(A2), 12, 526, 1020, false );
  // data.roll     = MapJoystickVal_others( analogRead(A3), 34, 514, 1020, true );
  drone_state_machine(state);
 
  Serial.print("Throttle: "); Serial.print(data.throttle);  Serial.print("    ");
  Serial.print("Yaw: ");      Serial.print(data.yaw);       Serial.print("    ");
  Serial.print("Pitch: ");    Serial.print(data.pitch);     Serial.print("    ");
  Serial.print("Roll: ");     Serial.print(data.roll);      Serial.print("    ");
  Serial.print("left bool = ");                             Serial.print(left_button);   //print start bool, check interrupt 2
  Serial.print("    ");
  Serial.print("right bool = ");                            Serial.print(right_button);   //print start bool, check interrupt 3
  Serial.print("    ");
  Serial.print("Drone State: ");                            Serial.print(state.droneState);
  Serial.print("\n"); 
 
  radio.write(&data, sizeof(MyData));
}

