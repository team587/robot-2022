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
  void AutoAim();
  void Stop();
  void SpeedCycle();
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
  double getCurrentTurretAngle();
  double getCurrentHoodAngle();
  void adjustHoodAngle();
  void adjustTurretAngle();
  void setShooterSpeed(double ShooterSpeed) {
    shooterSpeed = ShooterSpeed;
    m_shooterMotor1.Set(shooterSpeed);
  }
  
 private:
  
  rev::CANSparkMax m_shooterMotor1{canIDs::kShooterMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_shooterMotor2{canIDs::kShooterMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_hoodMotor{canIDs::kHoodMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_turningMotor{canIDs::kTurningMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::SparkMaxLimitSwitch m_turningLimitSwitch0;
  rev::SparkMaxLimitSwitch m_turningLimitSwitch180;

  rev::SparkMaxRelativeEncoder m_turretEncoder;
  frc::AnalogInput m_hoodAnalogInput;

  frc2::PIDController m_hoodPIDController{hoodP, hoodI, hoodD};
  frc2::PIDController m_turretPIDController{turretP, turretI, turretD};

  double shooterSpeed;
  double shooterSpeedH;
  double shooterSpeedM;
  double shooterSpeedL;
  double turningSpeed;
  double hoodAngle;
  double hoodVoltageOffset;
  double turretAngle;
  bool isRunning = false;
  bool hSpeed;
  bool mSpeed;
  bool lSpeed;
  bool noSpeed;

  double hoodP = 2.25;
  double hoodI = 0;
  double hoodD = 0.01;

  double turretP = 4.0;
  double turretI = 0.4;
  double turretD = 0.1;

  

};
