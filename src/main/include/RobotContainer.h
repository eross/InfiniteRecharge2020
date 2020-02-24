/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/Timer.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LiftSubsystem.h"
#include "subsystems/WheelOfFortuneSubsystem.h"
#include "commands/AutoVisionCommand.h"
#include "rev/CANSparkMax.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void SetDriveBreakMode(bool breakmode);
  bool enabled = false;
  bool lifty = false;
 private:
  std::string gameData;
  bool grabdataonce = false;
  char color;
  bool lifted = false;
  frc::SendableChooser<int> m_chooser;
  frc::Timer m_timer;
  DriveSubsystem m_drivesubsystem;
  ShooterSubsystem m_shootersubsystem;
  IntakeSubsystem m_intakesubsystem;
  LiftSubsystem m_liftsubsystem;
  WheelOfFortuneSubsystem m_wheeloffortunesubsystem;
  
  frc::Joystick m_driverController{DriveControllerConst::Controller};
  frc::Joystick m_operatorController{OperatorControllerConst::Controller};

  void ConfigureButtonBindings();
};
