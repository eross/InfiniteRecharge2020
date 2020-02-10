/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoVisionCommand.h"

AutoVisionCommand::AutoVisionCommand(std::function<void()> output, std::function<void()> feedpid, std::function<double()> vision, frc::DifferentialDrive* wheelSpeeds, std::initializer_list<frc2::Subsystem*> requirements) : wheelSpeeds(wheelSpeeds), vision(vision), feedpid(feedpid), output(output)
{
  AddRequirements(requirements);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoVisionCommand::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void AutoVisionCommand::Execute()
{
  timer.Start();
  if(timer.Get() <= 1)
  {
    feedpid();
    wheelSpeeds->ArcadeDrive(0, vision());
  }
  else
  {
    output();
  }
}

// Called once the command ends or is interrupted.
void AutoVisionCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoVisionCommand::IsFinished() { return timer.Get() > 3; }
