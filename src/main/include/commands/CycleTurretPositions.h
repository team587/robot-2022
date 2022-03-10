// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <subsystems/ShooterSubsystem.h>

class CycleTurretPositions
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 CycleTurretPositions> {
 public:
  CycleTurretPositions(ShooterSubsystem* shooter, bool turretLeft);

  void Initialize() override;

  ShooterSubsystem *m_shooterSubsystem;

  bool cycleLeft;
};
