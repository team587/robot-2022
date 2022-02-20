// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterFireCommand.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandBase.h>
#include <iostream>

ShooterFireCommand::ShooterFireCommand(ShooterSubsystem* shooterSubsystem) {
  std::cout << "Constructor Header\n";
  // Use addRequirements() here to declare subsystem dependencies.
  m_shooterSubsystem = shooterSubsystem;
  AddRequirements(m_shooterSubsystem);
  std::cout << "Constructor Exit\n";
}

// Called when the command is initially scheduled.
void ShooterFireCommand::Initialize() {
  std::cout << "Initialize\n";
  m_shooterSubsystem->Start();
}

// Called repeatedly when this Command is scheduled to run
void ShooterFireCommand::Execute() {
  std::cout << "Execute\n";
}

// Called once the command ends or is interrupted.
void ShooterFireCommand::End(bool interrupted) {
}

// Returns true when the command should end.
bool ShooterFireCommand::IsFinished() {
  return true;
}