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
#include <frc/shuffleboard/shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>


IntakeSubsystem::IntakeSubsystem() {
  //frc::Shuffleboard::GetTab("Intake").Add ("speed", IntakeSpeed);

  Retreat();
  m_autoOverride = false;
  m_intakeSpeed = 1.0;

}

void IntakeSubsystem::Periodic() {
  
  //m_intakeMotor.Set(m_autoOverride ? -1 : m_intakeSpeed);
}

void IntakeSubsystem::Toggle() {
  if (m_deployed) {
    Retreat();
  } else {
    Deploy();
  }

} 
void IntakeSubsystem::Deploy(){
  m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
  m_deployed = true;
}

void IntakeSubsystem::Retreat(){
  m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
  m_deployed = false;

}
void IntakeSubsystem::IntakeSpeed(double IntakeSpeed){
  m_intakeSpeed = IntakeSpeed;
  m_intakeMotor.Set(IntakeSpeed);
}
