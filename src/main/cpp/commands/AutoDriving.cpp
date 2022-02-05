// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriving.h"
#include "Constants.h"
#include "RobotContainer.h"
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

AutoDriving::AutoDriving(RobotContainer* container):m_container(container) {
  std::cout << "Constructor Header\n";
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_container->GetDriveSubsystem());
  std::cout << "Constructor Exit\n";
}

// Called when the command is initially scheduled.
void AutoDriving::Initialize() {
  std::cout << "Initialize\n";
  m_container->GetDriveSubsystem()->ResetOdometry(m_container->GetDriveSubsystem()->GetPose());
  std::cout << "Middle\n";
  Trajectory = m_trajectory.get_auto_trajectory();
  m_timer.Reset();
  m_timer.Start();
  std::cout << "Initialize Exit" << Trajectory << "\n";
}

// Called repeatedly when this Command is scheduled to run
void AutoDriving::Execute() {
  std::cout << "Execute\n";
  units::time::second_t time = m_timer.Get();
  std::cout << "GetTimer\n";
  PathPlannerTrajectory::PathPlannerState state = Trajectory->sample(time);
  std::cout << "Trajectory\n";
  frc::Pose2d RobotPose = m_container->GetDriveSubsystem()->GetPose();
  std::cout << "RobotPose\n";
  const auto adjustedSpeeds = controller.Calculate(RobotPose, state.pose, state.velocity, state.holonomicRotation);
  std::cout << "ControllerCalculate\n";
  //auto [fl, fr, bl, br] = m_container->GetDriveSubsystem()->kDriveKinematics.ToSwerveModuleStates(adjustedSpeeds);
  m_container->GetDriveSubsystem()->SetModuleStates(m_container->GetDriveSubsystem()->kDriveKinematics.ToSwerveModuleStates(adjustedSpeeds));
  std::cout << "Execute End\n";
}

// Called once the command ends or is interrupted.
void AutoDriving::End(bool interrupted) {
  m_timer.Stop();
  m_container->GetDriveSubsystem()->Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(0), true);
}

// Returns true when the command should end.
bool AutoDriving::IsFinished() {
  std::cout << "IsFinished\n";
  return m_timer.HasElapsed(Trajectory->getTotalTime());
}
