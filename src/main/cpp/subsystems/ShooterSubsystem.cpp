/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include <frc/Joystick.h>
//#include <frc/Relay.h>
#include <Constants.h>

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem(
    rev::CANSparkMax *shooterMotor1,
    rev::CANSparkMax *shooterMotor2,
    rev::CANSparkMax *hoodMotor,
    rev::CANSparkMax *turningMotor) {
      m_shooterMotor1 = shooterMotor1;
      m_shooterMotor2 = shooterMotor2;
      m_hoodMotor = hoodMotor;
      m_turningMotor = turningMotor;
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}


