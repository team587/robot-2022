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
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include "iostream"


ShooterSubsystem::ShooterSubsystem() :

#ifdef TURRET_SUBSYSTEM
      m_turningLimitSwitch0 (m_turningMotor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)),
      m_turningLimitSwitch180 (m_turningMotor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)),
      m_turretEncoder(m_turningMotor.GetEncoder()),
#endif

      m_hoodAnalogInput(0)
    {
      //shooterSpeed = .8;
      //shooterSpeedH = shooterSpeed;
      //shooterSpeedM = shooterSpeed - 0.1;
      //shooterSpeedL = shooterSpeed - 0.2;
      
      int count = 0;
      shooterSpeeds[count++] = 0;
      //shooterSpeeds[count++] = 0.7;
      //shooterSpeeds[count++] = 0.7;
      //shooterSpeeds[count++] = 0.6;//golden speed
      shooterSpeeds[count++] = 0.6;
      shooterSpeeds[count++] = 0.57; // auto speed
      shooterSpeeds[count++] = 0.3; // lower port
      
      count = 0;
      shooterAngles[count++] = 0;
      //shooterAngles[count++] = 10;
      //shooterAngles[count++] = 20;
      //shooterAngles[count++] = 20;
      shooterAngles[count++] = 30;
      shooterAngles[count++] = 30; // not used but for auto
      shooterAngles[count++] = 30; // lower port

      speedIndex = 0;
      dumpSpeed = false;

      turningSpeed = .1;
      hoodAngle = 0;
      turretAngle = 0;
      hoodVoltageOffset = 0.8;

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
      m_turretEncoder.SetPositionConversionFactor(1.121156 / ((281.0 / 30.0) * 10.0));
      m_turretEncoder.SetPosition(0);
      m_turretPIDController.Reset();
      m_turretPIDController.SetP(turretP);
      m_turretPIDController.SetI(turretI);
      m_turretPIDController.SetD(turretD);
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
      m_hoodPIDController.SetP(hoodP);
      m_hoodPIDController.SetI(hoodI);
      m_hoodPIDController.SetD(hoodD);
      m_hoodPIDController.SetTolerance(0.1);

      //frc::SmartDashboard::PutNumber("HoodAng...", 65);
      m_visionContainer.start();

  //autoShooterSpeed = 0;
  frc::SmartDashboard::PutNumber("PeriodicP", turretP);
  frc::SmartDashboard::PutNumber("PeriodicI", turretI);
  frc::SmartDashboard::PutNumber("PeriodicD", turretD);    
  //frc::SmartDashboard::PutNumber("PeriodicShoot", autoShooterSpeed);
  //frc::SmartDashboard::PutNumber("PeriodicHood", hoodAngle);
  
  
}

void ShooterSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //shooterSpeed = frc::SmartDashboard::GetNumber("Shooter Speed", shooterSpeed);

  frc::SmartDashboard::PutNumber("m_shooterMotor1", m_shooterMotor1.Get());
  frc::SmartDashboard::PutNumber("m_shooterMotor2", m_shooterMotor2.Get());

  //frc::Shuffleboard::GetTab("Shooter").("speed", shooterSpeed);
  frc::SmartDashboard::PutNumber("Hood", m_hoodAnalogInput.GetValue());

  if (m_driverController.GetRawButton(leftBumper)) {
   AutoAim();
    frc::SmartDashboard::PutBoolean("V Enabled", true);
    

  }else  {
     frc::SmartDashboard::PutBoolean("V Enabled", false);
  }
  
  // This is for characterization
  //shooterSpeed = frc::SmartDashboard::GetNumber("Shooter Speed", shooterSpeed);
  //m_shooterMotor1.Set(shooterSpeed);
  //hoodAngle = frc::SmartDashboard::GetNumber("Hood Des Angle", hoodAngle);
  adjustHoodAngle();

  if (dumpSpeed && speedIndex > 0) {
    m_shooterMotor1.Set(.3);
  } else {
    m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
  }
  //if(autoShooter) {
  //  m_shooterMotor1.Set(autoShooterSpeed);
  //}
#ifdef TURRET_SUBSYSTEM
  adjustTurretAngle();
#endif

  
  

  turretP = frc::SmartDashboard::GetNumber("PeriodicP", turretP);
  turretI = frc::SmartDashboard::GetNumber("PeriodicI", turretI);
  turretD = frc::SmartDashboard::GetNumber("PeriodicD", turretD);

  //m_turretPIDController.SetP(turretP);
  //m_turretPIDController.SetI(turretI);
  //m_turretPIDController.SetD(turretD);

  //hoodAngle = frc::SmartDashboard::GetNumber("PeriodicHood", hoodAngle);
  //setHoodAngle(hoodAngle);

  //autoShooterSpeed = frc::SmartDashboard::GetNumber("PeriodicShoot", autoShooterSpeed);
  
  //if(autoShooterSpeed != m_shooterMotor1.Get()) {
  //  m_shooterMotor1.Set(autoShooterSpeed);
  //}
  
}

//void ShooterSubsystem::SetSpeed(double speed) {
//  m_shooterMotor1.Set(speed);
//}
void ShooterSubsystem::start2() {
  autoShooter = false;
  speedIndex = 2;
  shooterSpeeds[speedIndex] = 0.67;
  m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
}

void ShooterSubsystem::Start() {
  autoShooter = false;
  speedIndex = 2;
  m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
  //m_shooterMotor1.Set(0.7);
  //isRunning = true;
  //hSpeed = true;
  //m_shooterMotor2.Set(shooterSpeed);
}

void ShooterSubsystem::AutoStart(double speed) {
  autoShooter = true;
  autoShooterSpeed = speed;
}

void ShooterSubsystem::SetLowSpeed() {
  autoShooter = false;
  speedIndex = 3;
  m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
}

void ShooterSubsystem::AutoAim() {

  frc::SmartDashboard::PutBoolean("AutoAimTarget", m_visionContainer.getHasTarget());
  if(m_visionContainer.getHasTarget()) {

    frc::SmartDashboard::PutNumber("AutoAimTurret", m_visionContainer.getTurretAngle(getCurrentTurretAngle()));
    frc::SmartDashboard::PutNumber("AutoAimHood", m_visionContainer.getHoodAngle(getCurrentHoodAngle()));
    frc::SmartDashboard::PutNumber("AutoAimShooter", m_visionContainer.getShooterSpeed(getCurrentHoodAngle()));
    frc::SmartDashboard::PutNumber("AutoAimDistance", m_visionContainer.getDistance(getCurrentHoodAngle()));

    setTurretAngle(m_visionContainer.getTurretAngle(getCurrentTurretAngle()));
    setHoodAngle(m_visionContainer.getHoodAngle(getCurrentHoodAngle()));
    AutoStart(m_visionContainer.getShooterSpeed(getCurrentHoodAngle()));
  }


  //m_visionContainer.getYaw();
/*
  auto result = m_camera.GetLatestResult();
      // wpi::outs() << "Camera is connected\n";
      frc::SmartDashboard::PutBoolean("has a target", result.HasTargets());
      if (result.HasTargets())
      {
        std::cout << "Has a target\n     ";
        // Does other calculations
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();
        // For Turret:
        #ifdef TURRET_SUBSYSTEM
        yaw = target.GetYaw();
        pitch = target.GetPitch();
        newTurretAngle = getCurrentTurretAngle();
        frc::SmartDashboard::PutNumber("AutoAim Orig Ang", newTurretAngle);
        frc::SmartDashboard::PutNumber("AutoAim Yaw", yaw);
        if (yaw > 1.0 || yaw < -1.0) {
          newTurretAngle = yaw + getCurrentTurretAngle();
          if (newTurretAngle > 180.0) {
            newTurretAngle = 180.0;
          } else if (newTurretAngle < 45.0) {
            newTurretAngle = 45.0;
          }
        }
        std::cout << newTurretAngle;
        std::cout << "yaw: " << yaw;
        frc::SmartDashboard::PutNumber("AutoAim New Ang", newTurretAngle);
        
        frc::SmartDashboard::PutNumber("AutoAim pitch", pitch);
        frc::SmartDashboard::PutNumber("AutoAim Ang", newTurretAngle);
        setTurretAngle(newTurretAngle);
        //turretAngle = m_visionContainer.getTurretAngle(getCurrentTurretAngle());
        frc::SmartDashboard::PutNumber("auto shooter speed", shooterSpeed);
        frc::SmartDashboard::PutNumber("AutoAim Hood Ang", getCurrentHoodAngle());
        //double angle = frc::SmartDashboard::GetNumber("HoodAng...", 65);
        double angle = 61;
        frc::SmartDashboard::PutNumber("distance: ", photonlib::PhotonUtils::CalculateDistanceToTarget(
          Camerapos::cam_height_meters, Camerapos::goal_height_meters, units::degree_t(angle),
          units::degree_t(pitch)).value());
        #endif
        
  }*/
}

void ShooterSubsystem::Stop() {
  autoShooter = false;
  speedIndex = 0;
  m_shooterMotor1.Set(shooterSpeeds[speedIndex]);
  //noSpeed = true;
  //m_shooterMotor2.Set(0);
}
void ShooterSubsystem::turnRight(){
  //m_turningMotor->Set(turningSpeed);
} 

void ShooterSubsystem::turnLeft(){
  //m_turningMotor->Set(turningSpeed * -1);
} 

void ShooterSubsystem::stopTurning(){
  //m_turningMotor->Set(0);
}

//
//I think we need to set hood angle and turret angle at end of functions
//
void ShooterSubsystem::adjustHoodAngle() {
  
  double currentAngle = m_hoodAnalogInput.GetVoltage();
  currentAngle -= hoodVoltageOffset;

  //90 is 360 divided by the gear reduction of the encoder (4)
  //5 is the voltage
  double angleToVoltage = 90.0 / 5.0;
  
  frc::SmartDashboard::PutNumber("Hood Cur Ang", currentAngle * angleToVoltage);
  frc::SmartDashboard::PutNumber("Hood Des Ang", hoodAngle);

  double output = m_hoodPIDController.Calculate(currentAngle, hoodAngle / angleToVoltage);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;
  frc::SmartDashboard::PutNumber("Hood Des Output", output);
  m_hoodMotor.Set(output);
}

#ifdef TURRET_SUBSYSTEM
void ShooterSubsystem::adjustTurretAngle() {
  
  double currentAngle = m_turretEncoder.GetPosition();

//  if(m_turningLimitSwitch0.Get()){
//    currentAngle = 0;
//    m_turretEncoder.SetPosition(currentAngle);
//  }

//  if(m_turningLimitSwitch180.Get()){
//    currentAngle = 0.5588;
//    m_turretEncoder.SetPosition(currentAngle);
//  }
  //double temp_p = frc::SmartDashboard::GetNumber("PValue", turretP);
  //double temp_i = frc::SmartDashboard::GetNumber("IValue", turretI);
  //double temp_d = frc::SmartDashboard::GetNumber("DValue", turretD);
  //m_turretPIDController.SetPID(temp_p,temp_i,temp_d);
  double metersToDegrees = 0.5588 / 180;

  frc::SmartDashboard::PutNumber("Turret Cur Ang Deg", currentAngle);
  frc::SmartDashboard::PutNumber("Turret Cur Ang", currentAngle / metersToDegrees);
  frc::SmartDashboard::PutNumber("Turret Des Ang", turretAngle);


  double output = m_turretPIDController.Calculate(currentAngle, turretAngle * metersToDegrees);
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;
  frc::SmartDashboard::PutNumber("Turret Des output", output);
  m_turningMotor.Set(output);
  //m_turningMotor.Set(0);
}

double ShooterSubsystem::getCurrentTurretAngle() {

  double currentAngle = m_turretEncoder.GetPosition();
  double metersToDegrees = 0.5588 / 180;

  return currentAngle / metersToDegrees;

}
#endif

double ShooterSubsystem::getCurrentHoodAngle() {

  double currentAngle = m_hoodAnalogInput.GetVoltage();
  currentAngle -= hoodVoltageOffset;
  //90 is 360 divided by the gear reduction of the encoder (4)
  //5 is the voltage
  double angleToVoltage = 90.0 / 5.0;
  
  return currentAngle * angleToVoltage;
}

void ShooterSubsystem::SetDumpMode(bool dump) { 
    frc::SmartDashboard::PutBoolean("Dump  Mode", dump);
    dumpSpeed = dump; 
  }

void ShooterSubsystem::SpeedCycle() {
  autoShooter = false;
  speedIndex++;
  speedIndex = speedIndex % 2; // slot 2 for auto i0 and 1  for teleop MAX_SETTINGS;
  hoodAngle = shooterAngles[speedIndex];
  frc::SmartDashboard::PutNumber("ShooterMode", speedIndex);
  
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