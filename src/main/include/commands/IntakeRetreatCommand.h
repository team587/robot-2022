// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>
#include "Trajectories.h"
#include "RobotContainer.h"

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
class IntakeRetreatCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeRetreatCommand> {
 public:
   IntakeRetreatCommand(IntakeSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  IntakeSubsystem *m_intakeSubsystem;

};
