/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include <frc/Joystick.h>
//#include <frc/Relay.h>
#include <Constants.h>

#include "subsystems/HopperSubsystem.h"

HopperSubsystem::HopperSubsystem(  
  rev::CANSparkMax * hopperMotor) {
  m_hopperMotor = hopperMotor;
  hopperSpeed = 1;

}
void HopperSubsystem::HopperStart(){
  m_hopperMotor->Set(hopperSpeed);
}
void HopperSubsystem::HopperReverse(){
  m_hopperMotor->Set(hopperSpeed * -1);
}
void HopperSubsystem::HopperStop(){
  m_hopperMotor->Set(0);
}
void HopperSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}




