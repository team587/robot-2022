// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/numbers>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/voltage.h>

#include "Constants.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               const int absoluteEncoderChannel,
               bool driveEncoderReversed, bool turningEncoderReversed,
               std::string name);

  frc::SwerveModuleState GetState();

  void SetDesiredState(const frc::SwerveModuleState& state);
  void SetDesiredAutoState(const frc::SwerveModuleState& state);
  

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(wpi::numbers::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::numbers::pi * 2.0);  // radians per second squared

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  std::string m_name;

  CANCoder m_absoluteEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  double turnP = -0.25;
  double turnI = 0.001;
  double turnD = 0;
  
  double driveP = 3.42;
  double driveI = 0;
  double driveD = 0; 
  
  double revDriveP = 3.42;
  double revDriveI = 0;
  double revDriveD = 0;
  double revDriveIZ = 0.001;
  double revDriveFF = 0.01;

  rev::SparkMaxRelativeEncoder m_drive_encoder;

  frc2::PIDController m_turningPIDController{turnP, turnI, turnD};
  frc2::PIDController m_drivePIDController{driveP, driveI, driveD};
  //rev::SparkMaxPIDController m_revDrivePIDController;
  frc::SimpleMotorFeedforward<units::meters> m_driveFeedForward{.086153_V, 2.4552_V / 1_mps};
};
