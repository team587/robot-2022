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
#include "VisionContainer.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public: 
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Start();
  //void AutoAim();
  void Stop();
  void SpeedCycle();
  //void turnRight();
  //void turnLeft();
  //void stopTurning();
  //void SetSpeed(double speed);
  void SetHoodAngle(double angle) { m_hoodAngle = angle; };
  double GetHoodAngle() { return m_hoodAngle; };

  //void SetDumpMode(bool dump);

#ifdef TURRET_SUBSYSTEM
  void SetTurretAngle(double TurretAngle) { m_turretAngle = TurretAngle; };
  double GetTurretAngle() { return m_turretAngle; };
  double GetCurrentTurretAngle();
  void AdjustTurretAngle();
#endif

  double GetCurrentHoodAngle();
  void AdjustHoodAngle();
  //void setShooterSpeed(double ShooterSpeed) {
  //  shooterSpeed = ShooterSpeed;
  //  m_shooterMotor1.Set(shooterSpeed);
  //}
  
 private:
  
  rev::CANSparkMax m_shooterMotor1{canIDs::kShooterMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_shooterMotor2{canIDs::kShooterMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_hoodMotor{canIDs::kHoodMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

#ifdef TURRET_SUBSYSTEM
  rev::CANSparkMax m_turningMotor{canIDs::kTurningMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::SparkMaxLimitSwitch m_turningLimitSwitch0;
  rev::SparkMaxLimitSwitch m_turningLimitSwitch180;

  rev::SparkMaxRelativeEncoder m_turretEncoder;
#endif

  frc::AnalogInput m_hoodAnalogInput;

  frc2::PIDController m_hoodPIDController{ m_hoodP, m_hoodI, m_hoodD };

#ifdef TURRET_SUBSYSTEM
  frc2::PIDController m_turretPIDController{ m_turretP, m_turretI, m_turretD };
#endif

  VisionContainer m_visionContainer;
  //frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  //double shooterSpeed;
  //double shooterSpeedH;
  //double shooterSpeedM;
  //double shooterSpeedL;
  const static int MAX_SETTINGS = 5;
  double m_shooterSpeeds[MAX_SETTINGS];
  double m_shooterAngles[MAX_SETTINGS];
  int m_speedIndex;
  
  double m_turningSpeed;
  double m_hoodAngle;
  double m_hoodVoltageOffset;
  double m_turretAngle;
  //bool isRunning = false;
  //bool hSpeed;
  //bool mSpeed;
  //bool lSpeed;
  //bool noSpeed;
  //bool dumpSpeed;

  double m_hoodP = 2.1;//2.25;
  double m_hoodI = 0;
  double m_hoodD = 0;//.01;

  double m_turretP = 1.2;//4.0;
  double m_turretI = 0;//.4;
  double m_turretD = 0;//.1;

};
