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
  
  PathPlannerTrajectory* get_auto_trajectory(int slot, int pathNum);
  PathPlannerTrajectory slot1_0;
  PathPlannerTrajectory slot1_1;
  PathPlannerTrajectory slot1_2;

  PathPlannerTrajectory slot2_0;
  PathPlannerTrajectory slot2_1;
  PathPlannerTrajectory slot2_2;

  PathPlannerTrajectory slot3_0;
  PathPlannerTrajectory slot3_1;
  PathPlannerTrajectory slot3_2;

  PathPlannerTrajectory slot4_0;
  PathPlannerTrajectory slot4_1;
  PathPlannerTrajectory slot4_2;

#endif

  
};
