/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/LiftSubsystem.h"

LiftSubsystem::LiftSubsystem()
{
    m_Lift = new frc::DoubleSolenoid(LiftConst::kLift0, LiftConst::kLift1);
    SetLiftPosition(true);
    m_Ratchet = new frc::DoubleSolenoid(3, 7);
    SetRatchetPosition(false);
    m_Winch.GetEncoder().SetPosition(0);
}

// This method will be called once per scheduler run
void LiftSubsystem::Periodic()
{
    //std::cout << m_Winch.GetEncoder().GetPosition() << std::endl;
}

void LiftSubsystem::SetWinchSpeed(double speed, double max_position, bool winch)
{
    if(speed == 0)
    {
        m_Winch.Set(0);
    }
    if(winch)
    {
        if(m_Winch.GetEncoder().GetPosition() < max_position)
        {
            m_Winch.Set(speed);
        }
        else
        {
            SetWinchIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_Winch.Set(0);
        }
    }
    else
    {
        if(m_Winch.GetEncoder().GetPosition() > max_position && speed <= 0)
        {
            m_Winch.Set(speed);
            SetWinchIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        }
        else if(speed <= 0)
        {
            m_Winch.Set(0);
        }
        else if(speed > 0)
        {
            SetWinchIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            m_Winch.Set(speed);
        }
    }
}

void LiftSubsystem::SetWinchIdleMode(rev::CANSparkMax::IdleMode idlemode)
{
    m_Winch.SetIdleMode(idlemode);
}

void LiftSubsystem::SetLiftPosition(bool position)
{
    m_Lift->Set(position ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);
}

void LiftSubsystem::SetRatchetPosition(bool position)
{
    m_Ratchet->Set(position ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);
}
