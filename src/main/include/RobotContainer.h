/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

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
#include "subsystems/DriveSubsystem.h"

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
 private:
  //frc::SendableChooser<frc2::Command*> m_chooser;
  DriveSubsystem m_drivesubsystem;
  
  frc::Joystick m_driverController{1};

  frc2::InstantCommand m_driveScaledSpeed{[this] { m_drivesubsystem.SetMaxOutput(0.8); }, {}};
  frc2::InstantCommand m_driveFullSpeed{[this] { m_drivesubsystem.SetMaxOutput(1); }, {}};

  void ConfigureButtonBindings();
};
