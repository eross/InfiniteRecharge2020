/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/WPILib.h>
#include <units/units.h>
#include <wpi/math>

namespace RobotMain
{
    const uint8_t PCM_ID = 0;
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
}

namespace DriveControllerConst
{
    const uint8_t
        Controller = 0,
        SpeedAxis = 1,
        RotateAxis = 4;
}

namespace OperatorControllerConst
{
    const uint8_t
        Controller = 1;
}

namespace DriveConst
{
    constexpr auto kTrackwidth = 0.6013_m;//0.6731_m;
    constexpr double kWheelDiameterMeters = 0.1524;
    constexpr double kencoderConstant = (1 / 10) * kWheelDiameterMeters * wpi::math::pi;

    const uint8_t
        leftLeadDeviceID = 1,
        leftFollowDeviceID = 2,
        rightLeadDeviceID = 4,
        rightFollowDeviceID = 3;
    constexpr auto ks = 0.17_V;
    constexpr auto kv = 2.66 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.493 * 1_V * 1_s * 1_s / 1_m;
    constexpr double kPDriveVel = 0.0000652;
    
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
}

namespace ShooterConst
{
    const uint8_t
        shooter0ID = 10,
        shooter1ID = 9,
        shooter2ID = 8,
        shooter3ID = 12,
        indexerID = 7,
        kSlotIdx = 0,
	    kPIDLoopIdx = 0,
	    kTimeoutMs = 30;
    const double
        kF = 0.0183,
        kP = 0.1,
        kI = 0,
        kD = 0;    
}

namespace IntakeConst
{
    const uint8_t
        kIntake = 11,
        kSlider0 = 0,
        kSlider1 = 4;
}

namespace LiftConst
{
    const uint8_t
        kLift0 = 1,
        kLift1 = 5,
        kWinch = 5;
}

namespace AutoConst
{
    constexpr auto kMaxSpeed = 2.2_mps;//1
    constexpr auto kMaxAcceleration = 2_mps_sq;//3

    constexpr double kRamseteB = 2;
    constexpr double kRamseteZeta = 0.7;
}


