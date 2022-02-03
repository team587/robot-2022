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
#include <rev/CANSparkMax.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public: 
  ShooterSubsystem( 
    rev::CANSparkMax *shooterMotor1,
    rev::CANSparkMax *shooterMotor2,
    rev::CANSparkMax *hoodMotor,
    rev::CANSparkMax *turningMotor);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Start();
  void Stop();
  
 private:
  
  rev::CANSparkMax *m_shooterMotor1;
  rev::CANSparkMax *m_shooterMotor2;
  rev::CANSparkMax *m_hoodMotor;
  rev::CANSparkMax *m_turningMotor;
  double shooterSpeed;



};
