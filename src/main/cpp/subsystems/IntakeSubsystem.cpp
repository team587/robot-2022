/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include <frc/Joystick.h>
//#include <frc/Relay.h>
#include <Constants.h>
#include "subsystems/IntakeSubsystem.h"


IntakeSubsystem::IntakeSubsystem(
  rev::CANSparkMax *intakeMotor,
  frc::Solenoid *intakeSolenoid) {

    m_intakeMotor = intakeMotor;
    m_intakeSolenoid = intakeSolenoid;
}

void IntakeSubsystem::Periodic() {

}
void IntakeSubsystem::Deploy(){
  m_intakeSolenoid->Set(true);
}

void IntakeSubsystem::Retreat(){
  m_intakeSolenoid->Set(false);

}
void IntakeSubsystem::IntakeSpeed(double IntakeSpeed){
  m_intakeMotor->Set(IntakeSpeed);
}
