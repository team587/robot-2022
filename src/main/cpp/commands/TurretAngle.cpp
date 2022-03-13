// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurretAngle.h"

TurretAngle::TurretAngle(double setDegrees, ShooterSubsystem* shooter) {
  // Use addRequirements() here to declare subsystem dependencies.


}

// Called when the command is initially scheduled.
void TurretAngle::Initialize() {
#ifdef TURRET_SUBSYSTEM
  //shooter->setTurretAngle(setDegrees);
#endif
}

// Called repeatedly when this Command is scheduled to run
void TurretAngle::Execute() {}

// Called once the command ends or is interrupted.
void TurretAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool TurretAngle::IsFinished() {
  return true;
}
