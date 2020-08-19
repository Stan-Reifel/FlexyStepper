
//      ******************************************************************
//      *                                                                *
//      *                 Header file for FlexyStepper.c                 *
//      *                                                                *
//      *               Copyright (c) S. Reifel & Co, 2014               *
//      *                                                                *
//      ******************************************************************


// MIT License
// 
// Copyright (c) 2014 Stanley Reifel & Co.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef FlexyStepper_h
#define FlexyStepper_h

#include <Arduino.h>
#include <stdlib.h>


//
// the FlexyStepper class
//
class FlexyStepper
{
  public:
    //
    // public functions
    //
    FlexyStepper();
    void connectToPins(byte stepPinNumber, byte directionPinNumber);

    void setStepsPerMillimeter(double motorStepPerMillimeter);
    double getCurrentPositionInMillimeters();
    void setCurrentPositionInMillimeters(double currentPositionInMillimeters);
    void setCurrentPositionInMillimeter(double currentPositionInMillimeter);
    void setSpeedInMillimetersPerSecond(double speedInMillimetersPerSecond);
    void setAccelerationInMillimetersPerSecondPerSecond(double accelerationInMillimetersPerSecondPerSecond);
    void setDecelerationInMillimetersPerSecondPerSecond(double decelerationInMillimetersPerSecondPerSecond);
    bool moveToHomeInMillimeters(long directionTowardHome, double speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin);
    void moveRelativeInMillimeters(double distanceToMoveInMillimeters);
    void setTargetPositionRelativeInMillimeters(double distanceToMoveInMillimeters);
    void moveToPositionInMillimeters(double absolutePositionToMoveToInMillimeters);
    void setTargetPositionInMillimeters(double absolutePositionToMoveToInMillimeters);
    double getTargetPositionInMillimeters();
    double getCurrentVelocityInMillimetersPerSecond();
   
    void setStepsPerRevolution(double motorStepPerRevolution);
    void setCurrentPositionInRevolutions(double currentPositionInRevolutions);
    double getCurrentPositionInRevolutions();
    void setSpeedInRevolutionsPerSecond(double speedInRevolutionsPerSecond);
    void setAccelerationInRevolutionsPerSecondPerSecond(double accelerationInRevolutionsPerSecondPerSecond);
    void setDecelerationInRevolutionsPerSecondPerSecond(double decelerationInRevolutionsPerSecondPerSecond);
    bool moveToHomeInRevolutions(long directionTowardHome, double speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin);
    void moveRelativeInRevolutions(double distanceToMoveInRevolutions);
    void setTargetPositionRelativeInRevolutions(double distanceToMoveInRevolutions);
    void moveToPositionInRevolutions(double absolutePositionToMoveToInRevolutions);
    void setTargetPositionInRevolutions(double absolutePositionToMoveToInRevolutions);
    double getCurrentVelocityInRevolutionsPerSecond();

    void setCurrentPositionInSteps(long currentPositionInSteps);
    long getCurrentPositionInSteps();
    void setSpeedInStepsPerSecond(double speedInStepsPerSecond);
    void setAccelerationInStepsPerSecondPerSecond(double accelerationInStepsPerSecondPerSecond);
    void setDecelerationInStepsPerSecondPerSecond(double decelerationInStepsPerSecondPerSecond);
    bool moveToHomeInSteps(long directionTowardHome, double speedInStepsPerSecond, long maxDistanceToMoveInSteps, int homeSwitchPin);
    void moveRelativeInSteps(long distanceToMoveInSteps);
    void setTargetPositionRelativeInSteps(long distanceToMoveInSteps);
    void moveToPositionInSteps(long absolutePositionToMoveToInSteps);
    void setTargetPositionInSteps(long absolutePositionToMoveToInSteps);
    long getTargetPositionInSteps();
    void setTargetPositionToStop();
    bool motionComplete();
    double getCurrentVelocityInStepsPerSecond(); 
    bool processMovement(void);    
    void emergencyStop(void);


  private:
    //
    // private functions
    //
    void DeterminePeriodOfNextStep();

 
    //
    // private member variables
    //
    byte stepPin;
    byte directionPin;
    double stepsPerMillimeter;
    double stepsPerRevolution;
    int directionOfMotion;
    long currentPosition_InSteps;
    long targetPosition_InSteps;
    double desiredSpeed_InStepsPerSecond;
    double desiredPeriod_InUSPerStep;
    double acceleration_InStepsPerSecondPerSecond;
    double acceleration_InStepsPerUSPerUS;
    double deceleration_InStepsPerSecondPerSecond;
    double deceleration_InStepsPerUSPerUS;
    double acceleration_periodOfSlowestStep_InUS;
    double acceleration_minimumPeriodForAStoppedMotion;
    double deceleration_periodOfSlowestStep_InUS;
    double deceleration_minimumPeriodForAStoppedMotion;
    double nextStepPeriod_InUS;
    unsigned long lastStepTime_InUS;
    double currentStepPeriod_InUS;
    bool emergency_stop;
};

// ------------------------------------ End ---------------------------------
#endif

