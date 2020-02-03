/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/units.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>
#include <vector>
#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  void Drive(double speed, double rotate);

  void TankDriveVolts(units::volt_t left, units::volt_t right);

  void SetDriveReversed(bool reversed) { setDriveReversed = reversed; }

  void ResetEncoders();

  double GetAverageEncoderDistance();

  rev::CANEncoder GetLeftEncoder();
  rev::CANEncoder GetRightEncoder();

  void SetMaxOutput(double maxOutput);

  double GetHeading();

  double GetTurnRate();

  frc::Pose2d GetPose();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  void ResetOdometry(frc::Pose2d pose);

  DriveSubsystem();

  void Periodic();
  void SetDriveBreakMode(rev::CANSparkMax::IdleMode idlemode)
  {    
    m_leftLeadMotor.SetIdleMode(idlemode);
    m_rightLeadMotor.SetIdleMode(idlemode);
    m_leftFollowMotor.SetIdleMode(idlemode);
    m_rightFollowMotor.SetIdleMode(idlemode);
  }
 private:
  bool setDriveReversed = false;
  rev::ColorSensorV3 m_colorSensor{RobotMain::i2cPort};
  rev::ColorMatch m_colorMatcher;
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

  rev::CANSparkMax m_leftLeadMotor{DriveConst::leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{DriveConst::rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{DriveConst::leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{DriveConst::rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_leftEncoder = m_leftLeadMotor.GetEncoder();
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();
  frc::SpeedControllerGroup m_leftMotors{m_leftLeadMotor, m_leftFollowMotor};

  frc::SpeedControllerGroup m_rightMotors{m_rightLeadMotor, m_rightFollowMotor};

  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  frc::ADXRS450_Gyro m_gyro;

  frc::DifferentialDriveOdometry m_odometry;
  
  const bool kGyroReversed = true;
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
