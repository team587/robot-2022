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
#include <rev/ColorSensorV3.h>

class HopperSubsystem : public frc2::SubsystemBase {
 public:
  HopperSubsystem(
    rev::CANSparkMax *hopperMotor);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  frc::Color kColorCodes[2] = {
    //Red
    frc::Color(.55, .33, .12),
    //Blue
    frc::Color(.17, .39, .44),
  };


  double kColorTolerance = .9;
  int GetColor();
  std::string ConvertColor(int colorIndex);

  void Periodic() override;

   void HopperStart();
   void HopperStop();
   void HopperReverse();


  
 private:
  rev::CANSparkMax *m_hopperMotor;
  rev::ColorSensorV3 m_colorSensor {frc::I2C::Port::kOnboard };
  double hopperSpeed;
  double tolerance;
  
};
