/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()
{
    m_Indexer = new VictorSPX(ShooterConst::indexerID);

    m_Shooter0 = new TalonSRX(ShooterConst::shooter0ID);
    m_Shooter1 = new TalonSRX(ShooterConst::shooter1ID);
    m_Shooter2 = new TalonSRX(ShooterConst::shooter2ID);

    m_Shooter0->Follow(*m_Shooter2);
    m_Shooter1->Follow(*m_Shooter2);
    m_Shooter0->SetInverted(InvertType::OpposeMaster);
    m_Shooter1->SetInverted(InvertType::OpposeMaster);

    m_Shooter2->ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder, ShooterConst::kPIDLoopIdx,
        ShooterConst::kTimeoutMs);

    m_Shooter2->SetSelectedSensorPosition(0, ShooterConst::kPIDLoopIdx, ShooterConst::kTimeoutMs);

    m_Shooter2->SetSensorPhase(false);

    m_Shooter2->ConfigNominalOutputForward(0, ShooterConst::kTimeoutMs);
    m_Shooter2->ConfigNominalOutputReverse(0, ShooterConst::kTimeoutMs);
    m_Shooter2->ConfigPeakOutputForward(1, ShooterConst::kTimeoutMs);
    m_Shooter2->ConfigPeakOutputReverse(0, ShooterConst::kTimeoutMs);

    m_Shooter2->Config_kF(ShooterConst::kPIDLoopIdx, ShooterConst::kF, ShooterConst::kTimeoutMs);
    m_Shooter2->Config_kP(ShooterConst::kPIDLoopIdx, ShooterConst::kP, ShooterConst::kTimeoutMs);
    m_Shooter2->Config_kI(ShooterConst::kPIDLoopIdx, ShooterConst::kI, ShooterConst::kTimeoutMs);
    m_Shooter2->Config_kD(ShooterConst::kPIDLoopIdx, ShooterConst::kD, ShooterConst::kTimeoutMs);
}

void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::SetShooterSpeed(double speed)
{
    m_Shooter2->Set(ControlMode::PercentOutput, speed);
}

void ShooterSubsystem::SetShooterRPM(double rpm)
{
    m_Shooter2->Set(ControlMode::Velocity, rpm);
}

void ShooterSubsystem::SetIndexerSpeed(double speed)
{
    m_Indexer->Set(ControlMode::PercentOutput, -speed);
}
