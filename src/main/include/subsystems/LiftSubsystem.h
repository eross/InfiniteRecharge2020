/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <ctre/Phoenix.h>

class LiftSubsystem : public frc2::SubsystemBase {
 public:
  LiftSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void SetLiftPosition(bool position);
  void SetRatchetPosition(bool position);
  void SetWinchSpeed(double speed, double max_position, bool winch);
  void SetWinchIdleMode(rev::CANSparkMax::IdleMode idlemode);
  double GetCurrentHeight();
  void ForceWinchSpeed(double speed);

 private:
  rev::CANSparkMax m_Winch{LiftConst::kWinch, rev::CANSparkMax::MotorType::kBrushless};
  frc::DoubleSolenoid* m_Lift;
  frc::DoubleSolenoid* m_Ratchet;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
