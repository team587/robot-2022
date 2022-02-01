/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include <frc/Joystick.h>
//#include <frc/Relay.h>
#include <Constants.h>

#include "subsystems/ClimberSubsystem.h"
ClimberSubsystem::ClimberSubsystem(
    rev::CANSparkMax *climberMotor,
    frc::DigitalInput *extendedDigitalInput,
    frc::DigitalInput *contractedDigitalInput) {
  m_climberMotor = climberMotor;
  m_contractedDigitalInput = contractedDigitalInput;
  m_extendedDigitalInput = extendedDigitalInput;
}


void ClimberSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}


