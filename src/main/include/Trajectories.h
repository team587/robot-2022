// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Constants.h>

#pragma once

#ifndef EXCLUDE_PATHPLANNER

#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

#endif

class trajectories {
 public:
  trajectories();

#ifndef EXCLUDE_PATHPLANNER
  
  PathPlannerTrajectory* get_auto_trajectory(int slot);
  PathPlannerTrajectory slot1;
  PathPlannerTrajectory slot2;
  PathPlannerTrajectory slot3;
  PathPlannerTrajectory slot4;

#endif

};
