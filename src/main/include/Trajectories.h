// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;
class trajectories {
 public:
  trajectories();

  PathPlannerTrajectory* get_auto_trajectory();
  PathPlannerTrajectory slot_two_first;
  PathPlannerTrajectory slot_three_second;
};
