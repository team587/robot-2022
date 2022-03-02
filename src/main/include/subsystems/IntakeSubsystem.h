/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <frc/Relay.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandHelper.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
IntakeSubsystem(
  rev::CANSparkMax *intakeMotor,
  frc::Solenoid *intakeSolenoid);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Deploy();
  void Retreat();
  void IntakeSpeed(double IntakeSpeed);
  
 private:
  
  rev::CANSparkMax *m_intakeMotor;
  frc::Solenoid *m_intakeSolenoid;
  

};
