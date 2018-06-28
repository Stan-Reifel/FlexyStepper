
//      ******************************************************************
//      *                                                                *
//      * Example shows how to make changes while the motor is in motion *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// The biggest benefit of FlexyStepper vs SpeedyStepper is that while the 
// motor is turning, you can change its speed or target position.  This 
// example shows how to make such changes while in motion.
//
// A limitations to consider is the code placed inside the "loop while 
// moving" needs to execute VERY fast.  Perhaps no longer than 0.05 
// milliseconds.  
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
  // start by resetting the motor's current positon to 0
  //
  stepper.setCurrentPositionInSteps(0);
  bool stopFlag = false;


  //
  // setup the motor so it rotates almost forever, Note: this command
  // does not start moving yet
  //
  stepper.setTargetPositionInSteps(2000000000);
  
  
  //
  // set the initial speed and acceleration rates for the stepper motor
  //
  stepper.setSpeedInStepsPerSecond(100);
  stepper.setAccelerationInStepsPerSecondPerSecond(100);


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
    // check if motor has moved past position 400, if so increase the speed
    //
    long currentPosition = stepper.getCurrentPositionInSteps();
    if (currentPosition == 400)
      stepper.setSpeedInStepsPerSecond(200);


    //
    // check if the user has pressed the "Rewind" button, if so return the  
    // motor to its starting position
    //
    if ((digitalRead(STOP_BUTTON_PIN) == LOW) && (stopFlag == false))
    {
      stepper.setTargetPositionInSteps(0);
      stopFlag = true;
    }
  }


  //
  // delay before starting again
  //
  delay(4000);
}

