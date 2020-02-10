/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <ctre/Phoenix.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetIntakeSpeed(double speed);
  void SetSliderPosition(bool position);

 private:
  TalonSRX* m_Intake;
  frc::DoubleSolenoid* m_Slider;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
