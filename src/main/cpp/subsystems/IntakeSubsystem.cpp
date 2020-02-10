/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem()
{
    m_Intake = new TalonSRX(IntakeConst::kIntake);
    m_Slider = new frc::DoubleSolenoid(IntakeConst::kSlider0, IntakeConst::kSlider1);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SetIntakeSpeed(double speed)
{
    m_Intake->Set(ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::SetSliderPosition(bool position)
{
    m_Slider->Set(position ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);
}
