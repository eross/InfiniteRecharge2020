/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/WheelOfFortuneSubsystem.h"

WheelOfFortuneSubsystem::WheelOfFortuneSubsystem()
{
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);

    SpinnySource = new SparkPIDSource(m_SpinnyBoi.GetEncoder());
    PIDBoi = new frc::PIDController(0.005, 0, 0, SpinnySource, &m_SpinnyBoi);
    PIDBoi->SetOutputRange(-.6, .6);
    PIDBoi->SetEnabled(false);
    m_SpinnyBoi.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_SpinnyBoi.GetEncoder().SetPositionConversionFactor(1);
    m_SpinnyBoi.GetEncoder().SetPosition(0);
    m_LiftyBoi = new frc::DoubleSolenoid(2,6);
}

// This method will be called once per scheduler run
void WheelOfFortuneSubsystem::Periodic()
{


//    std::cout << m_SpinnyBoi.GetEncoder().GetPosition() / 142.2<< std::endl;
}

void WheelOfFortuneSubsystem::SetSpeed(double speed)
{
    m_SpinnyBoi.Set(speed);
}

void WheelOfFortuneSubsystem::SetPosition(double position)
{
    PIDBoi->SetSetpoint(position * 142.2);
}

void WheelOfFortuneSubsystem::SetLiftyPosition(bool position)
{
    m_LiftyBoi->Set(position ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);
}

void WheelOfFortuneSubsystem::ResetPID()
{
    PIDBoi->SetSetpoint(0);
    PIDBoi->Reset();
    SpinnySource->Reset();
}

void WheelOfFortuneSubsystem::ToggleLifty()
{
    if(lifty)
    {
        lifty = false;
        SetLiftyPosition(false);
    }
    else
    {
        lifty = true;
        SetLiftyPosition(true);
    }
}

void WheelOfFortuneSubsystem::SetPIDEnabled(bool enabled)
{
    PIDBoi->SetEnabled(enabled);
}

char WheelOfFortuneSubsystem::GetColor()
{
  frc::Color detectedColor = m_colorSensor.GetColor();
  char color;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
  
  if (matchedColor == kBlueTarget) {
    color = 'B';
  } else if (matchedColor == kRedTarget) {
    color = 'R';
  } else if (matchedColor == kGreenTarget) {
    color = 'G';
  } else if (matchedColor == kYellowTarget) {
    color = 'Y';
  } else {
    color = 'U';
  }
  return color;
}