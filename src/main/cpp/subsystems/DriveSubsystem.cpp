// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <iostream>

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

      m_odometry{kDriveKinematics, m_NavX.GetRotation2d(), frc::Pose2d()} {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_NavX.GetRotation2d(), m_frontLeft.GetState(),
                    m_rearLeft.GetState(), m_frontRight.GetState(),
                    m_rearRight.GetState());
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
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
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
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
  GyroAngle = GyroAngle + wpi::numbers::pi * 2.0;
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
                           frc::Rotation2d(units::degree_t(GetHeading())));
}
