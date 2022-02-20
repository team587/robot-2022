// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
#include <frc/controller/HolonomicDriveController.h>
#include <frc/Timer.h>
#include "Trajectories.h"

using namespace pathplanner;


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ExtendClimberCommand
    : public frc2::CommandHelper<frc2::CommandBase, ExtendClimberCommand> {
 public:
   ExtendClimberCommand(ClimberSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  ClimberSubsystem *m_climberSubsystem;

};
