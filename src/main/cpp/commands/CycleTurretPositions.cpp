// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CycleTurretPositions.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
CycleTurretPositions::CycleTurretPositions(ShooterSubsystem* shooter, bool turretLeft) {
  // Use addRequirements() here to declare subsystem dependencies.
  //0, 90, 180

  m_shooterSubsystem = shooter;

  cycleLeft = turretLeft;


}

// Called when the command is initially scheduled.
void CycleTurretPositions::Initialize() {

#ifdef TURRET_SUBSYSTEM
   double turretAngle = m_shooterSubsystem->GetTurretAngle();

  if (cycleLeft) {
    if (turretAngle <= 90.0) {
      m_shooterSubsystem->SetTurretAngle(0.0);
    } else if (turretAngle <= 180.0) {
      m_shooterSubsystem->SetTurretAngle(90.0);
    }
  } else {
    if (turretAngle >= 90.0) {
      m_shooterSubsystem->SetTurretAngle(180.0);
    } else if (turretAngle >= 0.0) {
      m_shooterSubsystem->SetTurretAngle(90.0);
    }
  }
#endif
}

