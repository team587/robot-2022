// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <iostream>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/shuffleboard/shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "Constants.h" 

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{
          kFrontLeftDriveMotorPort,
          kFrontLeftTurningMotorPort,
          kFrontLeftAbsoluteEncoderPort,
          kFrontLeftDriveEncoderReversed,
          kFrontLeftTurningEncoderReversed,
          "LF"},

      m_rearLeft{
          kRearLeftDriveMotorPort,
          kRearLeftTurningMotorPort,
          kRearLeftAbsoluteEncoderPort,
          kRearLeftDriveEncoderReversed, 
          kRearLeftTurningEncoderReversed,
          "LR"},

      m_frontRight{
          kFrontRightDriveMotorPort,       
          kFrontRightTurningMotorPort,
          kFrontRightAbsoluteEncoderPort,
          kFrontRightDriveEncoderReversed, 
          kFrontRightTurningEncoderReversed,
          "RF"},

      m_rearRight{
          kRearRightDriveMotorPort,       
          kRearRightTurningMotorPort,
          kRearRightAbsoluteEncoderPort,
          kRearRightDriveEncoderReversed, 
          kRearRightTurningEncoderReversed,
          "RR"},
      
      m_speedController(1.0),
      m_odometry{kDriveKinematics, m_NavX.GetRotation2d(), frc::Pose2d()} {
        m_lastXSpeed = (units::meters_per_second_t)0.0;
        m_lastYSpeed = (units::meters_per_second_t)0.0;
        m_decelerate = (units::meters_per_second_t)0.01;

        //frc::Shuffleboard::GetTab("Drive").Add("decelerate", (double)m_decelerate);
      }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //m_odometry.Update(m_NavX.GetRotation2d(), m_frontLeft.GetState(),
  m_odometry.Update(frc::Rotation2d(units::radian_t(GetHeading())), m_frontLeft.GetState(),
                    m_rearLeft.GetState(), m_frontRight.GetState(),
                    m_rearRight.GetState());
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  

/*
  m_decelerate = (units::meters_per_second_t)(frc::Shuffleboard::GetTab("Drive")
    .Add("decelerate", (double)m_decelerate)
    .GetEntry()
    .GetDouble((double)m_decelerate));

  if(xSpeed == (units::meters_per_second_t)0.0 && m_lastXSpeed != (units::meters_per_second_t)0.0) {
    xSpeed = m_lastXSpeed > (units::meters_per_second_t)0.0 ? 
        m_lastXSpeed - m_decelerate < (units::meters_per_second_t)0.0 ? (units::meters_per_second_t)0.0 : m_lastXSpeed - m_decelerate 
      : 
        m_lastXSpeed + m_decelerate > (units::meters_per_second_t)0.0 ? (units::meters_per_second_t)0.0 : m_lastXSpeed + m_decelerate;
  }

  if(ySpeed == (units::meters_per_second_t)0.0 && m_lastYSpeed != (units::meters_per_second_t)0.0) {
    ySpeed = m_lastYSpeed > (units::meters_per_second_t)0.0 ? 
        m_lastYSpeed - m_decelerate < (units::meters_per_second_t)0.0 ? (units::meters_per_second_t)0.0 : m_lastYSpeed - m_decelerate 
      : 
        m_lastYSpeed + m_decelerate > (units::meters_per_second_t)0.0 ? (units::meters_per_second_t)0.0 : m_lastYSpeed + m_decelerate; 
  }
*/
  if (fabs((double)xSpeed) < 0.05) {
    xSpeed = (units::meters_per_second_t)0.0;
  }
  
  if (fabs((double)ySpeed) < 0.05) {
    ySpeed = (units::meters_per_second_t)0.0;
  }

  m_lastXSpeed = xSpeed;
  m_lastYSpeed = ySpeed;

  xSpeed = xSpeed / m_speedController;
  ySpeed = ySpeed / m_speedController;
  rot = rot / m_speedController;                        
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds( 
                          xSpeed, ySpeed, rot, m_NavX.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);

  //double maxspeed = (double)AutoConstants::kMaxSpeed;

  desiredStates[0].speed = -desiredStates[0].speed / 2.0;// / maxspeed;
  desiredStates[1].speed = -desiredStates[1].speed / 2.0;// / maxspeed; 
  desiredStates[2].speed = -desiredStates[2].speed / 2.0;// / maxspeed; 
  desiredStates[3].speed = -desiredStates[3].speed / 2.0;// / maxspeed; 

  m_frontLeft.SetDesiredAutoState(desiredStates[0]);
  m_frontRight.SetDesiredAutoState(desiredStates[1]);
  m_rearLeft.SetDesiredAutoState(desiredStates[2]);
  m_rearRight.SetDesiredAutoState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::radian_t DriveSubsystem::GetHeading() const {
  double GyroAngle = (double)m_NavX.GetRotation2d().Radians();
  GyroAngle = fmod(GyroAngle, wpi::numbers::pi * 2.0);
  if (GyroAngle < 0)
      GyroAngle = GyroAngle + wpi::numbers::pi * 2.0; // make sure 0 -2pi
  GyroAngle = GyroAngle;// - wpi::numbers::pi; // convert to -pi to pi
  return (units::radian_t)GyroAngle;

}

void DriveSubsystem::ZeroHeading() {
  m_NavX.Reset();
  //std::cout << "Here\n";
}

double DriveSubsystem::GetTurnRate() {
  return -m_NavX.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::radian_t(GetHeading())));
}
