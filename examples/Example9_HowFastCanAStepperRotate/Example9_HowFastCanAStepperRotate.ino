
//      ******************************************************************
//      *                                                                *
//      *       Example demoing how fast a stepper motor can rotate      *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// The top speed of a stepper motor is related to many factors.  The motor, 
// the load on the motor, the power supply voltage, and this library.
// 
// This library can generate a maximum of about 7,000 steps per second using an 
// Arduino Uno.  Assuming a system driving only one motor at a time, in full 
// step mode, with a 200 steps per rotation motor, the maximum speed is about 
// 35 RPS or 2100 RPM (most stepper motor can not go this fast).  Driving one 
// motor in half step mode, a maximum speed of 17 RPS or 1050 RPM can be reached.  
// In quarter step mode about 9 RPS or 525 RPM.  Running multiple motors at the 
// same time will reduce the maximum speed.  For example running two motors 
// will reduce the step rate by half or more.
//
// Smaller motors can typically spin faster than larger ones.  The best way  
// to evaluate a motor is by looking at its "torque curve".  Most of the 
// stepper motors sold by www.pololu.com have a data sheet on their website 
// showing a torque curve (motors sold on Amazon usually do not).
//
// For a library that can generate faster step rates, see:
//    https://github.com/Stan-Reifel/SpeedyStepper
//  
//
// Documentation for this library is at:
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


//
// create the stepper motor object
//
FlexyStepper stepper;



void setup() 
{
  //
  // setup the LED pin and enable print statements
  //
  pinMode(LED_PIN, OUTPUT);   
  Serial.begin(9600);


  //
  // connect and configure the stepper motor to its IO pins
  //
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
}



void loop() 
{
  //
  // Here will will try rotating at faster and faster speeds.  You can
  // see how the motor's torque is reduced as the speed increases by
  // pinching the shaft with your fingers.  Note: When the motor can't
  // go at the commanded speed, it simply stops.
  //
  // I have conducted all of these tests with 1x microstepping.
  //

  //
  // 100 steps/second
  //
  Serial.println("Testing 100 steps/second.");
  stepper.setSpeedInStepsPerSecond(100);
  stepper.setAccelerationInStepsPerSecondPerSecond(100);
  stepper.moveRelativeInSteps(200);
  delay(800);

  //
  // 200 steps/second
  //
  Serial.println("Testing 200 steps/second.");
  stepper.setSpeedInStepsPerSecond(200);
  stepper.setAccelerationInStepsPerSecondPerSecond(200);
  stepper.moveRelativeInSteps(400);
  delay(800);

  //
  // 400 steps/second
  //
  Serial.println("Testing 400 steps/second.");
  stepper.setSpeedInStepsPerSecond(400);
  stepper.setAccelerationInStepsPerSecondPerSecond(400);
  stepper.moveRelativeInSteps(800);
  delay(800);

  //
  // 800 steps/second
  // Note: My 12V NEAMA 17 motor with a 20V power supply stops here
  //
  Serial.println("Testing 800 steps/second.");
  stepper.setSpeedInStepsPerSecond(800);
  stepper.setAccelerationInStepsPerSecondPerSecond(800);
  stepper.moveRelativeInSteps(1600);
  delay(800);

  //
  // 1600 steps/second
  //
  Serial.println("Testing 1600 steps/second.");
  stepper.setSpeedInStepsPerSecond(1600);
  stepper.setAccelerationInStepsPerSecondPerSecond(1600); 
  stepper.moveRelativeInSteps(3200);
  delay(800);

  //
  // 3200 steps/second
  //
  Serial.println("Testing 3200 steps/second.");
  stepper.setSpeedInStepsPerSecond(3200);
  stepper.setAccelerationInStepsPerSecondPerSecond(3200); 
  stepper.moveRelativeInSteps(6400);
  delay(800);

  //
  // 6400 steps/second
  // Note: My 3.2V NEAMA 17 motor with a 20V power supply stops here
  //
  Serial.println("Testing 6400 steps/second.");
  stepper.setSpeedInStepsPerSecond(6400);
  stepper.setAccelerationInStepsPerSecondPerSecond(6400); 
  stepper.moveRelativeInSteps(12800);
  delay(800);


  Serial.println("");  
  delay(3000);
}



