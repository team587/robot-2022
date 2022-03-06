// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CycleHoodPositions.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
CycleHoodPositions::CycleHoodPositions(ShooterSubsystem* shooter, bool cycleUp) {
  // Use addRequirements() here to declare subsystem dependencies.

  double hoodAngle = shooter->getHoodAngle();

  if (cycleUp) {
    if (hoodAngle < 15.0) {
      shooter->setHoodAngle(15.0);
    } else if (hoodAngle < 30.0) {
      shooter->setHoodAngle(30.0);
    }
  } else {
    if (hoodAngle > 15.0) {
      shooter->setHoodAngle(15.0);
    } else if (hoodAngle > 0.0) {
      shooter->setHoodAngle(0.0);
    }
  }

}

// Called when the command is initially scheduled.
void CycleHoodPositions::Initialize() {

}