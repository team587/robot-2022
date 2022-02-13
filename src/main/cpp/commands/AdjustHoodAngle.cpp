// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AdjustHoodAngle.h"


AdjustHoodAngle::AdjustHoodAngle(double setDegrees, ShooterSubsystem* shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  shooter->setHoodAngle(setDegrees);
}

// Called when the command is initially scheduled.
void AdjustHoodAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AdjustHoodAngle::Execute() {}

// Called once the command ends or is interrupted.
void AdjustHoodAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AdjustHoodAngle::IsFinished() {
  return true;
}
