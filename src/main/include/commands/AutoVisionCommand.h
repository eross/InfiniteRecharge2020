/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <wpi/ArrayRef.h>
#include <functional>
#include <initializer_list>
#include <frc2/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/Subsystem.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <units/units.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoVisionCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoVisionCommand> {
  frc::DifferentialDrive* wheelSpeeds;
  std::function<double()> vision;
  std::function<void()> feedpid;
  std::function<void()> output;
  frc::Timer timer;
  double shoot_time = 4;
 public:
  AutoVisionCommand(std::function<void()> output, std::function<void()> feedpid, std::function<double()> vision, frc::DifferentialDrive* wheelSpeeds, std::initializer_list<frc2::Subsystem*> requirements);
  AutoVisionCommand(std::function<void()> output, std::function<void()> feedpid, std::function<double()> vision, frc::DifferentialDrive* wheelSpeeds, std::initializer_list<frc2::Subsystem*> requirements, double shoot_time);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
