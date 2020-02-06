/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

RobotContainer::RobotContainer()// : m_autonomousCommand(&m_subsystem)
{
  ConfigureButtonBindings();

  m_drivesubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drivesubsystem.Drive(
          m_driverController.GetRawAxis(DriveControllerConst::SpeedAxis),
          m_driverController.GetRawAxis(DriveControllerConst::RotateAxis) / 2);
    },
    {&m_drivesubsystem}));
  
  m_shootersubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_shootersubsystem.SetShooterSpeed(m_operatorController.GetRawAxis(2));
      m_shootersubsystem.SetIndexerSpeed(m_operatorController.GetRawAxis(2));
    },
    {&m_shootersubsystem}));
}

void RobotContainer::ConfigureButtonBindings()
{
    frc2::JoystickButton(&m_driverController, 6)
    .WhenPressed(&m_driveFullSpeed)
    .WhenReleased(&m_driveScaledSpeed);
}

void RobotContainer::SetDriveBreakMode(bool breakmode)
{
  if(breakmode)
  {
    m_drivesubsystem.SetDriveBreakMode(rev::CANSparkMax::IdleMode::kBrake);
  }
  else
  {
    m_drivesubsystem.SetDriveBreakMode(rev::CANSparkMax::IdleMode::kCoast);
  }
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics, 10_V);

  frc::TrajectoryConfig config(AutoConst::kMaxSpeed, AutoConst::kMaxAcceleration);
  config.SetKinematics(DriveConst::kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);
  
  auto testTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
    {frc::Translation2d(2.2098_m, 0.7018_m)},
    frc::Pose2d(3.5786_m, 0.7018_m, frc::Rotation2d(0_deg)),
    config);

  auto testTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(180_deg)),
    {frc::Translation2d(-2.2098_m, -0.7018_m)},
    frc::Pose2d(-3.5786_m, -0.7018_m, frc::Rotation2d(180_deg)),
    config);

  frc2::RamseteCommand ramseteCommand(
      testTrajectory, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  frc2::RamseteCommand ramseteCommand2(
      testTrajectory2, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand), frc2::InstantCommand([this] { m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(180_deg))); }, {}), frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(true); }, {}), std::move(ramseteCommand2),
      frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {}));
}
