/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

RobotContainer::RobotContainer()// : m_autonomousCommand(&m_subsystem)
{
  m_chooser.AddDefault("8 trench", 0);
  m_chooser.AddObject("steal", 1);
  m_chooser.AddObject("3 ball", 2);
  m_chooser.AddObject("7 steal", 3);
  m_chooser.AddDefault("5 trench", 4);
	frc::SmartDashboard::PutData("Auto Chooser" , &m_chooser);
  
	frc::SmartDashboard::PutNumber("RPM" , 0);
  ConfigureButtonBindings();
  
  m_drivesubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      //std::cout << m_operatorController.GetPOV(0) << std::endl;
      /*
      if(m_operatorController.GetPOV(0) > 0/*m_timer.GetMatchTime() < 30 && m_timer.GetMatchTime() > 26)
      {
        m_operatorController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1);
        m_operatorController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1);
      }
      else
      {
        m_operatorController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
        m_operatorController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);
      }*/
        m_drivesubsystem.Drive(
            m_driverController.GetRawAxis(DriveControllerConst::SpeedAxis) / (m_driverController.GetRawAxis(3) > .1 ? 1 : 1.2),
            m_driverController.GetRawButton(1) ?  m_drivesubsystem.GetLimeOutput()->GetOutput() :  (-m_driverController.GetRawAxis(DriveControllerConst::RotateAxis) / 1.5));
        //std::cout << m_drivesubsystem.GetLimeOutput()->GetOutput() << std::endl;
      if(!lifted)
      {
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
      }
    },
    {&m_drivesubsystem}));
    
  m_shootersubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(!lifted)
      {
        if(m_driverController.GetRawAxis(2) > .4)
        {
          m_shootersubsystem.Limelight(true);
        }
        if(m_driverController.GetRawAxis(2) < .4)
        {
          m_shootersubsystem.Limelight(false);
        }
        m_shootersubsystem.SetShooterRPM(m_operatorController.GetRawButton(5) ? m_shootersubsystem.GetDistanceToRPM()/*frc::SmartDashboard::GetNumber("RPM", 0)*/ : 0);
        m_shootersubsystem.SetIndexerSpeed(m_operatorController.GetRawButton(1) ? 1 : m_operatorController.GetRawButton(7) ? -1 : 0);
      }
    },
    {&m_shootersubsystem}));
    
  m_intakesubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      
      if(!lifted)
      {
        m_intakesubsystem.SetIntakeSpeed(abs(m_operatorController.GetRawAxis(3) - m_operatorController.GetRawAxis(2)) > .05 ? (m_operatorController.GetRawAxis(3) - m_operatorController.GetRawAxis(2)) : m_operatorController.GetRawButton(1) ? .3 : 0);
        m_intakesubsystem.SetSliderPosition(m_operatorController.GetRawButton(4) ? true : false);
      }
    },
    {&m_intakesubsystem}));

  m_liftsubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      
      if(!lifted)
      {
        if(m_operatorController.GetPOV(0) == 0)
        {
          m_liftsubsystem.SetLiftPosition(false);
        }
        else if(m_operatorController.GetRawButton(3))
        {
          m_liftsubsystem.SetLiftPosition(true);
          m_liftsubsystem.SetRatchetPosition(false);
        }
      }
      if(m_operatorController.GetRawButton(6))
      {
        lifted = true;
        m_liftsubsystem.SetLiftPosition(true);
        if(m_liftsubsystem.GetCurrentHeight() >= -35)
        {
          m_liftsubsystem.SetRatchetPosition(true);
        }
        m_liftsubsystem.SetWinchSpeed(.5, -30, true);
      }
      else if(m_operatorController.GetPOV(0) >= 0)
      {
        
        if(lifted)
        {
          if(m_operatorController.GetPOV(0) == 180)
          {
            m_liftsubsystem.SetRatchetPosition(true);
            m_liftsubsystem.ForceWinchSpeed(.5);
          }
          else
          {
            m_liftsubsystem.ForceWinchSpeed(0);
          }
        }
        if(!lifted)
        {
          m_liftsubsystem.SetRatchetPosition(false);
          m_liftsubsystem.SetWinchSpeed(m_operatorController.GetPOV(0) == 0 ? -.5 : m_operatorController.GetPOV(0) == 180 ? .3 : 0, -80, false);
        }
      }
      else
      {
        m_liftsubsystem.SetWinchSpeed(0, -1, false);
      }
      
      if(enabled)
      {
        m_liftsubsystem.SetWinchIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_liftsubsystem.SetWinchSpeed(0, -1, false);
        m_liftsubsystem.SetRatchetPosition(false);
        enabled = false;
      }
    },
    {&m_liftsubsystem}));

    
    m_wheeloffortunesubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(lifty)
      {
        m_wheeloffortunesubsystem.SetPosition(true);
      }

      if(m_operatorController.GetRawButtonPressed(2))
      {
        if(!grabdataonce)
        {
          gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
          grabdataonce = true;
        }
        m_wheeloffortunesubsystem.SetPIDEnabled(false);
      }

      if(m_operatorController.GetRawButton(2))
      {
        color = m_wheeloffortunesubsystem.GetColor();
        if(gameData.length() > 0)
        {
          switch (gameData[0])
          {
            case 'B' :
              switch(color)
              {
                case 'R' :
                  m_wheeloffortunesubsystem.SetSpeed(0);
                  break;
                default :
                  m_wheeloffortunesubsystem.SetSpeed(.1);
                  break;
              }
              break;
            case 'G' :
            switch(color)
              {
                case 'Y' :
                  m_wheeloffortunesubsystem.SetSpeed(0);
                  break;
                default :
                  m_wheeloffortunesubsystem.SetSpeed(.1);
                  break;
              }
              break;
            case 'R' :
            switch(color)
              {
                case 'B' :
                  m_wheeloffortunesubsystem.SetSpeed(0);
                  break;
                default :
                  m_wheeloffortunesubsystem.SetSpeed(.1);
                  break;
              }
              break;
            case 'Y' :
            switch(color)
              {
                case 'G' :
                  m_wheeloffortunesubsystem.SetSpeed(0);
                  break;
                default :
                  m_wheeloffortunesubsystem.SetSpeed(.1);
                  break;
              }
              break;
            default :
              std::cout << "unreadable color" << gameData[0] << std::endl;
              break;
          }
        } else {
          std::cout << "no game data" << std::endl;
        }
      }
      else
      {
        if(m_operatorController.GetRawButtonPressed(3))
        {
          m_wheeloffortunesubsystem.ResetPID();
          m_wheeloffortunesubsystem.SetPIDEnabled(true);
        }
        if(!m_operatorController.GetRawButton(3))
        {
          if(m_operatorController.GetRawButton(1))
          {
            m_wheeloffortunesubsystem.SetPIDEnabled(true);
            m_wheeloffortunesubsystem.SetPosition(-100);
          }
          else
          {
            m_wheeloffortunesubsystem.SetPIDEnabled(false);
            m_wheeloffortunesubsystem.SetSpeed(0);
          }
        }
        else
        {
          m_wheeloffortunesubsystem.SetPosition(4);
        }
      }


      if(m_operatorController.GetRawButtonPressed(8))
      {
        m_wheeloffortunesubsystem.ToggleLifty();
      }
    },
    {&m_wheeloffortunesubsystem}));

      
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
  auto Trajectory0 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
    {frc::Translation2d(50_in, -16_in)},
    frc::Pose2d(228_in, -16_in, frc::Rotation2d(0_deg)),
    config);

  auto Trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(180_deg)),
    {frc::Translation2d(-50.0_in, 0.0_in)},
    frc::Pose2d(-132_in, 0.0_in, frc::Rotation2d(180_deg)),
    config);
/*old
  auto Trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(0_deg)),
    {frc::Translation2d(20_in, -20.0_in), frc::Translation2d(13_in, -40.0_in)},
    frc::Pose2d(13_in, -135.0_in, frc::Rotation2d(-90_deg)),
    config);

  auto Trajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(13.0_in, -135.0_in, frc::Rotation2d(-90_deg)),
    {frc::Translation2d(13_in, -150.0_in)},
    frc::Pose2d(13_in, -226.0_in, frc::Rotation2d(-90_deg)),
    config);
*/
  auto Trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(0_deg)),
    {frc::Translation2d(20_in, -20.0_in)},
    frc::Pose2d(13_in, -135.0_in, frc::Rotation2d(-93_deg)),
    config);

  auto Trajectory3 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(13.0_in, -135.0_in, frc::Rotation2d(-93_deg)),
    {frc::Translation2d(13_in, -150.0_in)},
    frc::Pose2d(13_in, -226.0_in, frc::Rotation2d(-90_deg)),
    config);

  auto Trajectory4 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(-90_deg)),
    {frc::Translation2d(-1_in, -70.0_in)},
    frc::Pose2d(-2_in, -126.0_in, frc::Rotation2d(-95_deg)),
    config);

  auto Trajectory5 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(0_deg)),
    {frc::Translation2d(20_in, 20.0_in)},
    frc::Pose2d(13_in, 104.0_in, frc::Rotation2d(93_deg)),
    config);

  auto Trajectory6 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(93_deg)),
    {frc::Translation2d(-20_in, 35.0_in)},
    frc::Pose2d(-40_in, 70.0_in, frc::Rotation2d(130_deg)),
    config);

  auto Trajectory7 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(0_deg)),
    {frc::Translation2d(6_in, 0.0_in)},
    frc::Pose2d(12_in, 0.0_in, frc::Rotation2d(0_deg)),
    config);

  auto Trajectory8 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(93_deg)),
    {frc::Translation2d(-20_in, 20.0_in)},
    frc::Pose2d(-130_in, 50.0_in, frc::Rotation2d(110_deg)),
    config);

  auto Trajectory9 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(93_deg)),
    {frc::Translation2d(-1_in, 5.0_in)},
    frc::Pose2d(-2_in, 26.0_in, frc::Rotation2d(68_deg)),
    config);

  auto Trajectory10 = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0.0_in, 0.0_in, frc::Rotation2d(68_deg)),
    {frc::Translation2d(2_in, 10.0_in)},
    frc::Pose2d(1_in, 26.0_in, frc::Rotation2d(120_deg)),
    config);


  frc2::RamseteCommand ramseteCommand(
      Trajectory0, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  frc2::RamseteCommand ramseteCommand2(
      Trajectory1, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  frc2::RamseteCommand ramseteCommand3(
      Trajectory2, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  frc2::RamseteCommand ramseteCommand4(
      Trajectory3, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  frc2::RamseteCommand ramseteCommand5(
      Trajectory4, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

  frc2::RamseteCommand ramseteCommand6(
      Trajectory5, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});
    
    frc2::RamseteCommand ramseteCommand7(
      Trajectory6, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

    frc2::RamseteCommand ramseteCommand8(
      Trajectory7, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

    frc2::RamseteCommand ramseteCommand9(
      Trajectory8, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

    frc2::RamseteCommand ramseteCommand10(
      Trajectory9, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});

    frc2::RamseteCommand ramseteCommand11(
      Trajectory10, [this]() { return m_drivesubsystem.GetPose(); },
      frc::RamseteController(AutoConst::kRamseteB, AutoConst::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(DriveConst::ks, DriveConst::kv, DriveConst::ka),
      DriveConst::kDriveKinematics,
      [this] { return m_drivesubsystem.GetWheelSpeeds(); },
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConst::kPDriveVel, 0, 0), 
      [this](auto left, auto right) { m_drivesubsystem.TankDriveVolts(left, right); },
      {&m_drivesubsystem});


  /*
  return new frc2::SequentialCommandGroup(
    frc2::InstantCommand([this] { 
        m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
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
    */
    if(m_chooser.GetSelected() == 0)
    {
      return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      std::move(ramseteCommand3),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true);
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
      frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
      std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),
      frc2::InstantCommand([this] { 
          m_shootersubsystem.SetIndexerSpeed(0);
          m_shootersubsystem.SetShooterRPM(0); }, {&m_shootersubsystem}),
      frc2::InstantCommand([this] { 
        m_intakesubsystem.SetIntakeSpeed(1);
        m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      std::move(ramseteCommand4),
      frc2::InstantCommand([this] { 
        m_intakesubsystem.SetIntakeSpeed(1);
        m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      frc2::InstantCommand([this] { 
        m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(units::degree_t(m_drivesubsystem.GetHeading())))); }, {&m_drivesubsystem}),
      frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(true); }, {&m_drivesubsystem}),
      std::move(ramseteCommand5),
      frc2::InstantCommand([this] { 
        m_intakesubsystem.SetIntakeSpeed(1);
        m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      frc2::InstantCommand([this] { 
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
      frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
      std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),

        frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {})
      );
    }
    else if(m_chooser.GetSelected() == 1)
    {
      return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
        std::move(ramseteCommand6),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(0);
          m_intakesubsystem.SetSliderPosition(false); }, {&m_intakesubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(units::degree_t(m_drivesubsystem.GetHeading())))); }, {&m_drivesubsystem}),
        frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(true); }, {&m_drivesubsystem}),
        std::move(ramseteCommand7),
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true);
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(.5); }, {&m_intakesubsystem}),
        std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem}, 8)),

        frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {})
      );
    }
    else if(m_chooser.GetSelected() == 2)
    {
      return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true); }, {&m_shootersubsystem}),
        std::move(ramseteCommand8),
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true);
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(.5); }, {&m_intakesubsystem}),
        std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem}, 8)),

        frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {})
      );
    }
    else if(m_chooser.GetSelected() == 3)
    {
      return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
        std::move(ramseteCommand6),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(0);
          m_intakesubsystem.SetSliderPosition(false); }, {&m_intakesubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(units::degree_t(m_drivesubsystem.GetHeading())))); }, {&m_drivesubsystem}),
        frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(true); }, {&m_drivesubsystem}),
        std::move(ramseteCommand9),
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true);
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(.5); }, {&m_intakesubsystem}),
        std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(units::degree_t(m_drivesubsystem.GetHeading())))); }, {&m_drivesubsystem}),
        frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(false); }, {&m_drivesubsystem}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
        std::move(ramseteCommand10),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(0);
          m_intakesubsystem.SetSliderPosition(false); }, {&m_intakesubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.ResetOdometry(frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(units::degree_t(m_drivesubsystem.GetHeading())))); }, {&m_drivesubsystem}),
        frc2::InstantCommand([this] { m_drivesubsystem.SetDriveReversed(true); }, {&m_drivesubsystem}),
        std::move(ramseteCommand11),
        frc2::InstantCommand([this] { 
          m_shootersubsystem.SetIndexerSpeed(0);
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(.5); }, {&m_intakesubsystem}),
        std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),
        frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {})
      );
    }
    else if(m_chooser.GetSelected() == 4)
    {
      return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      std::move(ramseteCommand3),
        frc2::InstantCommand([this] { 
          m_intakesubsystem.SetIntakeSpeed(1);
          m_intakesubsystem.SetSliderPosition(true); }, {&m_intakesubsystem}),
      frc2::InstantCommand([this] { 
          m_shootersubsystem.Limelight(true);
          m_shootersubsystem.SetShooterRPM(m_shootersubsystem.GetDistanceToRPM()); }, {&m_shootersubsystem}),
      frc2::InstantCommand([this] { 
          m_drivesubsystem.GetLimePID()->Enable(); }, {}),
      std::move(AutoVisionCommand([this]() { return m_shootersubsystem.SetIndexerSpeed(1); }, [this]() { return m_drivesubsystem.GetLimeSource()->SetInput(m_drivesubsystem.GetTargetCenter()); }, [this]() { return m_drivesubsystem.GetLimeOutput()->GetOutput(); }, &m_drivesubsystem.m_drive, {&m_drivesubsystem, &m_shootersubsystem})),
      frc2::InstantCommand([this] { 
          m_shootersubsystem.SetIndexerSpeed(0);
          m_shootersubsystem.SetShooterRPM(0); }, {&m_shootersubsystem}),
        frc2::InstantCommand([this] { m_drivesubsystem.TankDriveVolts(0_V, 0_V); }, {})
      );
    }
}
