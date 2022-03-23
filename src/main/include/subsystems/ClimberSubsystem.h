/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Joystick.h>
#include <frc/Relay.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandHelper.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem(
    rev::CANSparkMax *climberMotor,
    frc::DigitalInput *extendedDigitalInput,
    frc::DigitalInput *contractedDigitalInput);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //void ClimberForward();
  //void ClimberBackward();
  //void ClimberStop();

  //bool ClimberExtended();
  //bool ClimberContracted();

  void ClimberSpeed(double speed);
  
  void ToggleClimberStart() { m_startClimb = !m_startClimb; };
  bool GetClimberStart() { return m_startClimb; };
  
 private:
  
    rev::CANSparkMax *m_climberMotor;
    frc::DigitalInput *m_extendedDigitalInput;
    frc::DigitalInput *m_contractedDigitalInput;
    //double m_speed;
    bool m_startClimb;
    //frc::Joystick m_coDriverController{OIConstants::kCoDriverControllerPort};
};
