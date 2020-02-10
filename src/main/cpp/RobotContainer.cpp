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
          m_driverController.GetRawAxis(DriveControllerConst::SpeedAxis) / (m_driverController.GetRawAxis(3) > .1 ? 1 : 1.2),
          m_driverController.GetRawButton(1) ?  m_drivesubsystem.GetLimeOutput()->GetOutput() :  (-m_driverController.GetRawAxis(DriveControllerConst::RotateAxis) / 1.5));
      //std::cout << m_drivesubsystem.GetLimeOutput()->GetOutput() << std::endl;
      m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter());
      if(m_driverController.GetRawButtonPressed(1))
      {
        std::cout << "enable" << std::endl;
        m_drivesubsystem.GetLimePID()->Reset();
        m_drivesubsystem.GetLimePID()->Enable();
      }
      else if(m_driverController.GetRawButtonReleased(1))
      {
        m_drivesubsystem.GetLimePID()->Disable();
      }
    },
    {&m_drivesubsystem}));
    
  m_shootersubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_shootersubsystem.SetShooterRPM(m_operatorController.GetRawButton(5) ? m_shootersubsystem.GetDistanceToRPM() : 0);
      m_shootersubsystem.SetIndexerSpeed(m_operatorController.GetRawButton(1) ? 1 : m_operatorController.GetRawButton(8) ? -1 : 0);
    },
    {&m_shootersubsystem}));
    
  m_intakesubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_intakesubsystem.SetIntakeSpeed(abs(m_operatorController.GetRawAxis(3) - m_operatorController.GetRawAxis(2)) > .05 ? (m_operatorController.GetRawAxis(3) - m_operatorController.GetRawAxis(2)) : m_operatorController.GetRawButton(1) ? .3 : 0);
      m_intakesubsystem.SetSliderPosition(m_operatorController.GetRawButton(4) ? true : false);
    },
    {&m_intakesubsystem}));
    
}

void RobotContainer::ConfigureButtonBindings()
{
      
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
    frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
    {frc::Translation2d(50_in, -16_in)},
    frc::Pose2d(228_in, -16_in, frc::Rotation2d(0_deg)),
    config);

  auto testTrajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(180_deg)),
    {frc::Translation2d(-50.0_in, 0.0_in)},
    frc::Pose2d(-132_in, 0.0_in, frc::Rotation2d(180_deg)),
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
;

  return new frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] { 
        m_shootersubsystem.SetShooterRPM(5100/*m_shootersubsystem.GetDistanceToRPM()*/); }, {&m_shootersubsystem}),
    frc2::InstantCommand([this] { 
        m_drivesubsystem.GetLimePID()->Enable(); }, {}),
    std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),
    frc2::InstantCommand([this] { 
        m_shootersubsystem.SetIndexerSpeed(0);
        m_shootersubsystem.SetShooterRPM(0); }, {&m_shootersubsystem}),
    frc2::InstantCommand([this] { 
        m_intakesubsystem.SetIntakeSpeed(1);
        m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { 
        m_intakesubsystem.SetIntakeSpeed(0);
        m_intakesubsystem.SetSliderPosition(false); }, {&m_intakesubsystem}),
    frc2::InstantCommand([this] { m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(180_deg))); }, {&m_drivesubsystem}),
    frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(true); }, {&m_drivesubsystem}),
    std::move(ramseteCommand2),
    frc2::InstantCommand([this] { 
        m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
    frc2::InstantCommand([this] { 
        m_drivesubsystem.GetLimePID()->Enable(); }, {}),
    std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),

    frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {}));
}
