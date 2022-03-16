// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int absoluteEncoderChannel,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed,
                           std::string name)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_name(name),
      m_absoluteEncoder(absoluteEncoderChannel),
      m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed),
      m_drive_encoder(m_driveMotor.GetEncoder())//, 
      //m_revDrivePIDController(m_driveMotor.GetPIDController())
  {
      
  m_absoluteEncoder.SetPositionToAbsolute();

    m_driveMotor.RestoreFactoryDefaults();
    m_driveMotor.SetInverted(m_reverseDriveEncoder);
    m_driveMotor.SetSmartCurrentLimit(50);
    m_driveMotor.SetSecondaryCurrentLimit(80);
    m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_driveMotor.EnableVoltageCompensation(12);
    //m_drive_encoder = &m_driveMotor.GetEncoder();

    m_drive_encoder.SetPositionConversionFactor(0.319 / 6.12);
    m_drive_encoder.SetVelocityConversionFactor((0.319 / 6.12)/60.0); // wheel circumfrence meters / gear reduction
  /*
    m_revDrivePIDController.SetOutputRange(-1.0, 1.0);
    m_revDrivePIDController.SetP(revDriveP);
    m_revDrivePIDController.SetI(revDriveI);
    m_revDrivePIDController.SetD(revDriveD);
    m_revDrivePIDController.SetIZone(revDriveIZ);
    m_revDrivePIDController.SetFF(revDriveFF);
*/
    m_turningMotor.RestoreFactoryDefaults();
    m_turningMotor.SetInverted(m_reverseTurningEncoder);
    m_turningMotor.SetSmartCurrentLimit(50);
    m_turningMotor.SetSecondaryCurrentLimit(80);

    m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100);
    m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 100);   
    m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 100);
    m_driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100);
    m_driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);
    m_driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 100);   
    m_absoluteEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);

    frc::SmartDashboard::PutNumber("PValue", turnP);
    frc::SmartDashboard::PutNumber("IValue", turnI);
    frc::SmartDashboard::PutNumber("DValue", turnD);




  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  //m_driveEncoder.SetDistancePerPulse(
      //ModuleConstants::kDriveEncoderDistancePerPulse);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::numbers::pi)
  // divided by the encoder resolution.
  //m_turningEncoder.SetDistancePerPulse(
      //ModuleConstants::kTurningEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.

    m_turningPIDController.Reset();
    m_turningPIDController.SetTolerance(0.1);

  m_turningPIDController.EnableContinuousInput(
      -wpi::numbers::pi, wpi::numbers::pi);
}

frc::SwerveModuleState SwerveModule::GetState() {
  double angle = m_absoluteEncoder.GetAbsolutePosition();
  angle = angle * (wpi::numbers::pi / 180.0) - wpi::numbers::pi;
  return {units::meters_per_second_t{m_drive_encoder.GetVelocity()},
  //return {units::meters_per_second_t{0},

          frc::Rotation2d(units::radian_t(angle))};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  double angle = m_absoluteEncoder.GetAbsolutePosition();
  angle = angle * (wpi::numbers::pi / 180.0) - wpi::numbers::pi;
  //const auto state = frc::SwerveModuleState::Optimize(
      //referenceState, units::radian_t(angle));
  const auto state = referenceState;
  // Calculate the drive output from the drive PID controller.
    //const auto driveOutput = m_drivePIDController.Calculate(
    //m_drive_encoder.GetVelocity(), state.speed.value());
    //const auto driveFeedforward = m_driveFeedForward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  //auto turnOutput = m_turningPIDController.Calculate(
      //units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

  //frc::Shuffleboard::GetTab("Swerve").Add (m_name + " TurnAngle", state.angle.Radians().to<double>());
  //frc::Shuffleboard::GetTab("Swerve").Add (m_name + " CurAngle", angle);
  //frc::Shuffleboard::GetTab("Swerve").Add (m_name +" Speed", state.speed.to<double>());

  //frc::SmartDashboard::PutNumber(m_name + " TurnAngle", state.angle.Radians().to<double>());
  //frc::SmartDashboard::PutNumber(m_name + " CurAngle", angle);
  //frc::SmartDashboard::PutNumber(m_name + " Speed", state.speed.to<double>());
  //double temp_p = frc::SmartDashboard::GetNumber("PValue", turnP);
  //double temp_i = frc::SmartDashboard::GetNumber("IValue", turnI);
  //double temp_d = frc::SmartDashboard::GetNumber("DValue", turnD);

  //m_turningPIDController.SetPID(temp_p, temp_i, temp_d);
  double output = m_turningPIDController.Calculate(angle, state.angle.Radians().to<double>());
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  // Set the motor outputs.
  m_driveMotor.Set(referenceState.speed.to<double>());
  //m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  //m_revDrivePIDController.SetReference(referenceState.speed.to<double>() * AutoConstants::kMaxSpeed.to<double>(), rev::ControlType::kVelocity);
  
  //m_driveMotor.Set(0);
  m_turningMotor.Set(output);
}

void SwerveModule::SetDesiredAutoState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  double angle = m_absoluteEncoder.GetAbsolutePosition();
  angle = angle * (wpi::numbers::pi / 180.0) - wpi::numbers::pi;
  //const auto state = frc::SwerveModuleState::Optimize(
      //referenceState, units::radian_t(angle));
  const auto state = referenceState;
  // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(
    m_drive_encoder.GetVelocity(), state.speed.value());
    const auto driveFeedforward = m_driveFeedForward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  //auto turnOutput = m_turningPIDController.Calculate(
      //units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

  frc::SmartDashboard::PutNumber(m_name + " TurnAngle", state.angle.Radians().to<double>());
  frc::SmartDashboard::PutNumber(m_name + " CurAngle", angle);
  frc::SmartDashboard::PutNumber(m_name + " Speed", state.speed.to<double>());
  //double temp_p = frc::SmartDashboard::GetNumber("PValue", turnP);
  //double temp_i = frc::SmartDashboard::GetNumber("IValue", turnI);
  //double temp_d = frc::SmartDashboard::GetNumber("DValue", turnD);

  //m_turningPIDController.SetPID(temp_p, temp_i, temp_d);
  double output = m_turningPIDController.Calculate(angle, state.angle.Radians().to<double>());
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  // Set the motor outputs.
  //m_driveMotor.Set(referenceState.speed.to<double>());
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  //m_revDrivePIDController.SetReference(referenceState.speed.to<double>() * AutoConstants::kMaxSpeed.to<double>(), rev::ControlType::kVelocity);
  
  //m_driveMotor.Set(0);
  m_turningMotor.Set(output);
}

void SwerveModule::ResetEncoders() {
  //m_driveMotor.GetEncoder().SetPosition(0);
  //m_turningMotor.GetEncoder().SetPosition(0);
  m_absoluteEncoder.SetPosition(0);
}
