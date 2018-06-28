
//      ******************************************************************
//      *                                                                *
//      *    Example shows how to move while testing other conditions    *
//      *    at the same time                                             *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// This sketch shows how you can do other things while the motor is running.  
// Examples of "other things" might be to:  1) Decelerate to a stop once a 
// button is press.  2) Turn on a solenoid when the motor moves past 
// position 850.  3) Flash a strobe every 200 steps.  4) Run continuously as 
// long as a temperature is below 125 degrees.
//
// A limitations to consider is the code you run while the stepper is moving
// needs to execute VERY fast.  Perhaps no longer than 0.05 milliseconds.  
//
//  
// Documentation at:
//         https://github.com/Stan-Reifel/FlexyStepper
//
//
// The motor must be connected to the Arduino with a driver board having a 
// "Step and Direction" interface.  It's VERY important that you set the 
// motor current first!  Read the driver board's documentation to learn how.

// ***********************************************************************


#include <FlexyStepper.h>


//
// pin assignments
//
const int LED_PIN = 13;
const int MOTOR_STEP_PIN = 3;
const int MOTOR_DIRECTION_PIN = 4;
const int STOP_BUTTON_PIN = 9;


//
// create the stepper motor object
//
FlexyStepper stepper;



void setup() 
{
  //
  // setup the LED pin, stop button pin and enable print statements
  //
  pinMode(LED_PIN, OUTPUT);   
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);


  //
  // connect and configure the stepper motor to its IO pins
  //
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
}



void loop() 
{
  //
  // set the speed and acceleration rates for the stepper motor
  //
  stepper.setSpeedInStepsPerSecond(100);
  stepper.setAccelerationInStepsPerSecondPerSecond(100);


  //
  // set the motor's current positon to 0 and turn off the LED
  //
  stepper.setCurrentPositionInSteps(0);
  digitalWrite(LED_PIN, LOW);
  bool stopFlag = false;


  //
  // setup the motor so that it will rotate 2000 steps, Note: this 
  // command does not start moving yet
  //
  stepper.setTargetPositionInSteps(2000);
  

  //
  // now execute the move, looping until the motor has finished
  //
  while(!stepper.motionComplete())
  {
    //
    // Note: The code added to this loop must execute VERY fast.  
    // Perhaps no longer than 0.05 milliseconds.
    //
    
    //
    // process motor steps
    //
    stepper.processMovement();

    
    //
    // check if motor has moved past position 400, if so turn On the LED
    //
    if (stepper.getCurrentPositionInSteps() == 400)
      digitalWrite(LED_PIN, HIGH);


    //
    // check if the user has pressed the "Stop" button, if so decelerate to a stop
    //
    if ((digitalRead(STOP_BUTTON_PIN) == LOW) && (stopFlag == false))
    {
      stepper.setTargetPositionToStop();
      stopFlag = true;
    }
  }


  //
  // delay before starting again
  //
  delay(4000);
}

