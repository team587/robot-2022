// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ShooterSubsystem.h"
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>
#include "Trajectories.h"


#ifndef EXCLUDE_PATHPLANNER

using namespace pathplanner;

#endif

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShooterFireCommand
    : public frc2::CommandHelper<frc2::CommandBase, ShooterFireCommand> {
 public:
   ShooterFireCommand(ShooterSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  ShooterSubsystem *m_shooterSubsystem;

};
