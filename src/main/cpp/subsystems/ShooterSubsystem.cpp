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
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <subsystems/HopperSubsystem.h>
#include <frc/DriverStation.h>

ShooterSubsystem::ShooterSubsystem() :

#ifdef TURRET_SUBSYSTEM
      m_turningLimitSwitch0 (m_turningMotor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)),
      m_turningLimitSwitch180 (m_turningMotor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)),
      m_turretEncoder(m_turningMotor.GetEncoder()),
#endif

      m_hoodAnalogInput(0)
    {
      int count = 0;
      m_shooterSpeeds[count++] = 0;
      m_shooterSpeeds[count++] = 0.7;
      m_shooterSpeeds[count++] = 0.7;
      m_shooterSpeeds[count++] = 0.6;
      m_shooterSpeeds[count++] = 0.6;
      
      count = 0;
      m_shooterAngles[count++] = 0;
      m_shooterAngles[count++] = 10;
      m_shooterAngles[count++] = 20;
      m_shooterAngles[count++] = 20;
      m_shooterAngles[count++] = 30;

      m_speedIndex = 0;
      //dumpSpeed = false;

      m_turningSpeed = .1;
      m_hoodAngle = 0;
      m_turretAngle = 0;
      m_hoodVoltageOffset = 0.8;

      //frc::Shuffleboard::GetTab("Shooter").Add("speed", shooterSpeed);
    
      m_shooterMotor1.SetInverted(true);
      m_shooterMotor2.Follow(m_shooterMotor1, true);
      Stop();

#ifdef TURRET_SUBSYSTEM
      m_turningMotor.RestoreFactoryDefaults();
      m_turningMotor.SetSmartCurrentLimit(30);
      m_turningMotor.SetSecondaryCurrentLimit(50);
      m_turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_turningMotor.EnableVoltageCompensation(12);

      m_turningLimitSwitch0.EnableLimitSwitch(true);
      m_turningLimitSwitch180.EnableLimitSwitch(true);

      //In meters
      //1.121156 is the perimeter of the turret gear.
      //281 is the number of teeth on the turret gear.
      //30 is the number of teeth on the turret driving gear.
      //7 is the gear reduction of the vex planetary.
      m_turretEncoder.SetPositionConversionFactor(1.121156 / (281.0 / 30.0 * 7.0));
      m_turretEncoder.SetPosition(0);
      m_turretPIDController.Reset();
      m_turretPIDController.SetP(m_turretP);
      m_turretPIDController.SetI(m_turretI);
      m_turretPIDController.SetD(m_turretD);
      m_turretPIDController.SetTolerance(0.1);
#endif

      m_hoodMotor.RestoreFactoryDefaults();
      m_hoodMotor.SetSmartCurrentLimit(30);
      m_hoodMotor.SetSecondaryCurrentLimit(50);
      m_hoodMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
      m_hoodMotor.EnableVoltageCompensation(12);

      //rev::SparkMaxLimitSwitch m_hoodLimitSwitch0 = m_hoodMotor->GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
      //rev::SparkMaxLimitSwitch m_hoodLimitSwitch180 = m_hoodMotor->GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
      //m_hoodLimitSwitch0.EnableLimitSwitch(true);
      //m_hoodLimitSwitch180.EnableLimitSwitch(true);

      //m_hoodEncoder.SetPositionConversionFactor(8);

      m_hoodPIDController.Reset();
      m_hoodPIDController.SetP(m_hoodP);
      m_hoodPIDController.SetI(m_hoodI);
      m_hoodPIDController.SetD(m_hoodD);
      m_hoodPIDController.SetTolerance(0.1);

      //m_visionContainer.start();
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //shooterSpeed = frc::SmartDashboard::GetNumber("Shooter Speed", shooterSpeed);

  //frc::SmartDashboard::PutNumber("m_shooterMotor1", m_shooterMotor1.Get());
  //frc::SmartDashboard::PutNumber("m_shooterMotor2", m_shooterMotor2.Get());

  //frc::Shuffleboard::GetTab("Shooter").("speed", shooterSpeed);
  //frc::SmartDashboard::PutNumber("Hood", m_hoodAnalogInput.GetValue());

  //if (m_driverController.GetRawButton(leftBumper)) {
   //AutoAim();
  //}

  

  /*if (dumpSpeed && speedIndex > 0) {
    m_shooterMotor1.Set(.3);
  } else {
    m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
  }*/
 /* AdjustHoodAngle();
#ifdef TURRET_SUBSYSTEM
  AdjustTurretAngle();
#endif
*/
}

//void ShooterSubsystem::SetSpeed(double speed) {
//  m_shooterMotor1.Set(speed);
//}


void ShooterSubsystem::Start() {
  m_speedIndex = 1;
  m_shooterMotor1.Set(m_shooterSpeeds[m_speedIndex]);
  //isRunning = true;
  //hSpeed = true;
  //m_shooterMotor2.Set(shooterSpeed);
}
/*
void ShooterSubsystem::AutoAim(){

  double currentAngle = m_hoodAnalogInput.GetVoltage();
  currentAngle -= hoodVoltageOffset;
  double angleToVoltage = 90.0 / 5.0;
  double angle = currentAngle * angleToVoltage;
  hoodAngle = m_visionContainer.getHoodAngle(angle);
  // For Turret:
#ifdef TURRET_SUBSYSTEM
  turretAngle = m_visionContainer.getTurretAngle(angle);
  //frc::Shuffleboard::GetTab("Vision").Add("Shooter mcGavin Speed", shooterSpeed);
#endif
  // For wheels
  shooterSpeed = m_visionContainer.getShooterSpeed(angle);
  double distance = m_visionContainer.getDistance(angle);

  //frc::Shuffleboard::GetTab("Vision").Add("Distance", distance);
  //frc::Shuffleboard::GetTab("Vision").Add("Hood Angle", hoodAngle);
  //frc::Shuffleboard::GetTab("Vision").Add("Turret Angle", turretAngle);
  //frc::Shuffleboard::GetTab("Vision").Add("Current Angle", currentAngle);
  //frc::Shuffleboard::GetTab("Vision").Add("Enabled", m_driverController.GetRawButton(leftBumper));

}*/

void ShooterSubsystem::Stop() {
  m_speedIndex = 0;
  m_shooterMotor1.Set(m_shooterSpeeds[m_speedIndex]);
  //noSpeed = true;
  //m_shooterMotor2.Set(0);
}
/*
void ShooterSubsystem::turnRight(){
  //m_turningMotor->Set(turningSpeed);
} 

void ShooterSubsystem::turnLeft(){
  //m_turningMotor->Set(turningSpeed * -1);
} 

void ShooterSubsystem::stopTurning(){
  //m_turningMotor->Set(0);
}
*/
//
//I think we need to set hood angle and turret angle at end of functions
//
void ShooterSubsystem::AdjustHoodAngle() {
  
  double currentAngle = m_hoodAnalogInput.GetVoltage();
  currentAngle -= m_hoodVoltageOffset;

  //90 is 360 divided by the gear reduction of the encoder (4)
  //5 is the voltage
  double angleToVoltage = 90.0 / 5.0;
  
  frc::SmartDashboard::PutNumber("Hood Cur Ang", currentAngle * angleToVoltage);
  frc::SmartDashboard::PutNumber("Hood Des Ang", m_hoodAngle);

  double output = m_hoodPIDController.Calculate(currentAngle, m_hoodAngle / angleToVoltage);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;
  frc::SmartDashboard::PutNumber("Hood Des Output", output);
  m_hoodMotor.Set(output);
}

#ifdef TURRET_SUBSYSTEM
void ShooterSubsystem::AdjustTurretAngle() {
  
  double currentAngle = m_turretEncoder.GetPosition();

//  if(m_turningLimitSwitch0.Get()){
//    currentAngle = 0;
//    m_turretEncoder.SetPosition(currentAngle);
//  }

//  if(m_turningLimitSwitch180.Get()){
//    currentAngle = 0.5588;
//    m_turretEncoder.SetPosition(currentAngle);
//  }
  
  double metersToDegrees = 0.5588 / 180;

  frc::SmartDashboard::PutNumber("Turret Cur Ang", currentAngle / metersToDegrees);
  frc::SmartDashboard::PutNumber("Turret Des Ang", m_turretAngle);


  double output = m_turretPIDController.Calculate(currentAngle, m_turretAngle * metersToDegrees);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;
  frc::SmartDashboard::PutNumber("Turret Des output", output);
  m_turningMotor.Set(output);
  //m_turningMotor.Set(0);
}

double ShooterSubsystem::GetCurrentTurretAngle() {

  double currentAngle = m_turretEncoder.GetPosition();
  double metersToDegrees = 0.5588 / 180;

  return currentAngle / metersToDegrees;

}
#endif

double ShooterSubsystem::GetCurrentHoodAngle() {

  double currentAngle = m_hoodAnalogInput.GetVoltage();
  currentAngle -= m_hoodVoltageOffset;
  //90 is 360 divided by the gear reduction of the encoder (4)
  //5 is the voltage
  double angleToVoltage = 90.0 / 5.0;
  
  return currentAngle * angleToVoltage;
}
/*
void ShooterSubsystem::SetDumpMode(bool dump) { 
    frc::SmartDashboard::PutBoolean("Dump  Mode", dump);
    dumpSpeed = dump; 
  }
*/
void ShooterSubsystem::SpeedCycle() {
  m_speedIndex++;
  m_speedIndex = m_speedIndex % MAX_SETTINGS;
  m_hoodAngle = m_shooterAngles[m_speedIndex];
  m_shooterMotor1.Set(m_shooterSpeeds[m_speedIndex]);
  frc::SmartDashboard::PutNumber("ShooterMode", m_speedIndex);
  
  //if(speedIndex == 3) {
  //  speedIndex = 0;
  //}
  //m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
  
  
  /*if(noSpeed) {
    noSpeed = false;
    hSpeed = true;
    m_shooterMotor1.Set(shooterSpeedH);
  } else if(hSpeed) {
    hSpeed = false;
    mSpeed = true;
    m_shooterMotor1.Set(shooterSpeedM);
  } else if(mSpeed) {
    mSpeed = false;
    lSpeed = true;
    m_shooterMotor1.Set(shooterSpeedL);
  } else if(lSpeed) {
    lSpeed = false;
    hSpeed = true;
    m_shooterMotor1.Set(shooterSpeedH);
  }*/
}