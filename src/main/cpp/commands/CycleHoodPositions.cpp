// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CycleHoodPositions.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandBase.h>
#include <iostream>
// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
CycleHoodPositions::CycleHoodPositions(ShooterSubsystem* shooter, bool hoodUp) {
  // Use addRequirements() here to declare subsystem dependencies.

  m_shooterSubsystem = shooter;

  cycleUp = hoodUp;

}

// Called when the command is initially scheduled.
void CycleHoodPositions::Initialize() {
    double hoodAngle = m_shooterSubsystem->GetHoodAngle();

  if (cycleUp) {
    if (hoodAngle < 15.0) {
      m_shooterSubsystem->SetHoodAngle(15.0);
    } else if (hoodAngle < 30.0) {
      m_shooterSubsystem->SetHoodAngle(30.0);
    }
  } else {
    if (hoodAngle > 15.0) {
      m_shooterSubsystem->SetHoodAngle(15.0);
    } else if (hoodAngle > 0.0) {
      m_shooterSubsystem->SetHoodAngle(0.0);
    }
  }
}
