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
#include <frc/controller/PIDController.h>
#include <frc/AnalogInput.h>
#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public: 
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Start();
  void Stop();
  void turnRight();
  void turnLeft();
  void stopTurning();
  void setHoodAngle(double angle) {
    hoodAngle = angle;
  };
  double getHoodAngle() {
    return hoodAngle;
  };
  void setTurretAngle(double TurretAngle) {
    turretAngle = TurretAngle;
  };
  double getTurretAngle() {
    return turretAngle;
  };
  void adjustHoodAngle();
  void adjustTurretAngle();
  
 private:
  
  rev::CANSparkMax m_shooterMotor1{canIDs::kShooterMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_shooterMotor2{canIDs::kShooterMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_hoodMotor{canIDs::kHoodMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_turningMotor{canIDs::kTurningMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::SparkMaxRelativeEncoder m_turretEncoder;
  frc::AnalogInput m_hoodAnalogInput;

  frc2::PIDController m_hoodPIDController{hoodP, hoodI, hoodD};
  frc2::PIDController m_turretPIDController{turretP, turretI, turretD};

  double shooterSpeed;
  double turningSpeed;
  double hoodAngle;
  double turretAngle;

  double hoodP = 0;
  double hoodI = 0;
  double hoodD = 0;

  double turretP = 0;
  double turretI = 0;
  double turretD = 0;

  

};
