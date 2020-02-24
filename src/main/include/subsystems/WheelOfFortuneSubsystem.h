/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "SparkPIDSource.h"
#include "Constants.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

class WheelOfFortuneSubsystem : public frc2::SubsystemBase {
 public:
  WheelOfFortuneSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetSpeed(double speed);
  void SetPosition(double position);
  void SetLiftyPosition(bool position);
  void ToggleLifty();
  void ResetPID();
  void SetPIDEnabled(bool enabled);
  char GetColor();

 private:
  rev::ColorSensorV3 m_colorSensor{RobotMain::i2cPort};
  rev::ColorMatch m_colorMatcher;
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

  bool lifty = false;
  SparkPIDSource* SpinnySource;
  frc::PIDController* PIDBoi;
  rev::CANSparkMax m_SpinnyBoi{WheelOfFortuneConst::kSpinny, rev::CANSparkMax::MotorType::kBrushless};
  frc::DoubleSolenoid* m_LiftyBoi;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
