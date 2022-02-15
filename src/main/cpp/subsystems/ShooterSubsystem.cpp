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
    rev::CANSparkMax *turningMotor):
     m_turningMotor(turningMotor),
     m_hoodMotor(hoodMotor),
     m_turret_encoder(m_turningMotor->GetEncoder()),
     m_hood_encoder(m_hoodMotor->GetEncoder())
    {
      m_shooterMotor1 = shooterMotor1;
      m_shooterMotor2 = shooterMotor2;
      shooterSpeed = 1;
      shooterMotor2->Follow(*m_shooterMotor1, true);
      //shooterMotor2->SetInverted(true);
      turningSpeed = 1;

      hoodAngle = 0;
      turretAngle = 0;

      m_turningMotor->RestoreFactoryDefaults();
      m_turningMotor->SetSmartCurrentLimit(30);
      m_turningMotor->SetSecondaryCurrentLimit(50);
      m_turningMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_turningMotor->EnableVoltageCompensation(12);
      rev::SparkMaxLimitSwitch m_turningLimitSwitch0 = m_turningMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
      rev::SparkMaxLimitSwitch m_turningLimitSwitch180 = m_turningMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
      m_turningLimitSwitch0.EnableLimitSwitch(true);
      m_turningLimitSwitch180.EnableLimitSwitch(true);

      m_turret_encoder.SetPositionConversionFactor(8);

      m_turretPIDController.Reset();
      m_turretPIDController.SetTolerance(0.1);

      m_hoodMotor->RestoreFactoryDefaults();
      m_hoodMotor->SetSmartCurrentLimit(30);
      m_hoodMotor->SetSecondaryCurrentLimit(50);
      m_hoodMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_hoodMotor->EnableVoltageCompensation(12);
      rev::SparkMaxLimitSwitch m_hoodLimitSwitch0 = m_hoodMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
      rev::SparkMaxLimitSwitch m_hoodLimitSwitch180 = m_hoodMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
      m_hoodLimitSwitch0.EnableLimitSwitch(true);
      m_hoodLimitSwitch180.EnableLimitSwitch(true);

      m_hood_encoder.SetPositionConversionFactor(8);

      m_hoodPIDController.Reset();
      m_hoodPIDController.SetTolerance(0.1);

}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  adjustHoodAngle();
  adjustTurretAngle();
}

void ShooterSubsystem::Start() {
  m_shooterMotor1->Set(shooterSpeed);
  m_shooterMotor2->Set(shooterSpeed);
}

void ShooterSubsystem::Stop() {
  m_shooterMotor1->Set(0);
  m_shooterMotor2->Set(0);
}
void ShooterSubsystem::turnRight(){
  m_turningMotor->Set(turningSpeed);
} 

void ShooterSubsystem::turnLeft(){
  m_turningMotor->Set(turningSpeed * -1);
} 

void ShooterSubsystem::stopTurning(){
  m_turningMotor->Set(0);
}

void ShooterSubsystem::adjustHoodAngle() {
  rev::SparkMaxLimitSwitch m_hoodLimitSwitch0 = m_hoodMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
  rev::SparkMaxLimitSwitch m_hoodLimitSwitch180 = m_hoodMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
  double currentAngle = m_hood_encoder.GetPosition();

  if(m_hoodLimitSwitch0.Get()){
    currentAngle = 0;
    m_hood_encoder.SetPosition(0);
  }

  if(m_hoodLimitSwitch180.Get()){
    currentAngle = 180;
    m_hood_encoder.SetPosition(180);
  }

  double output = m_hoodPIDController.Calculate(currentAngle, hoodAngle);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  m_hoodMotor->Set(output);

}

void ShooterSubsystem::adjustTurretAngle() {
  rev::SparkMaxLimitSwitch m_turningLimitSwitch0 = m_turningMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
  rev::SparkMaxLimitSwitch m_turningLimitSwitch180 = m_turningMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
  double currentAngle = m_turret_encoder.GetPosition();

  if(m_turningLimitSwitch0.Get()){
    currentAngle = 0;
    m_turret_encoder.SetPosition(0);
  }

  if(m_turningLimitSwitch180.Get()){
    currentAngle = 180;
    m_turret_encoder.SetPosition(180);
  }

  double output = m_turretPIDController.Calculate(currentAngle, turretAngle);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  m_turningMotor->Set(output);
}