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

namespace DriveConst
{
    constexpr auto kTrackwidth = 0.66314_m;//0.6731_m;
    constexpr double kWheelDiameterMeters = 0.1524;
    constexpr double kencoderConstant = (1 / 10.42) * kWheelDiameterMeters * wpi::math::pi;

    const uint8_t
        leftLeadDeviceID = 16,
        leftFollowDeviceID = 15,
        rightLeadDeviceID = 14,
        rightFollowDeviceID = 13;
    constexpr auto ks = 0.145_V;
    constexpr auto kv = 2.72 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.467 * 1_V * 1_s * 1_s / 1_m;
    constexpr double kPDriveVel = 0.000615;
    
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
}

namespace AutoConst
{
    constexpr auto kMaxSpeed = 3_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;

    constexpr double kRamseteB = 2;
    constexpr double kRamseteZeta = 0.7;
}


