/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <math.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <ctre/Phoenix.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetShooterSpeed(double speed);
  void SetShooterRPM(double rpm);
  void SetIndexerSpeed(double speed);
  double GetTargetDistance()
  {
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double angle = table->GetNumber("ty",0.0);
    double distance = (90.75 - 15) / tan((double)(18.23 + angle) / 180 * M_PI);
    return distance;
  }
  double GetDistanceToRPM()
  {
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double dist = GetTargetDistance();
    if(table->GetNumber("tv",0.0) == 0)
    {
      return 1500;
    }
    else
    {
      double target = 7110.378 - 20.36336 * dist + 0.05301945*(dist * dist);
      return target;
    }
    //return 5580.768 - 12.57303 * dist + 0.03979932*(dist * dist);
  }
  void Limelight(bool light)
  {
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("ledMode", light ? 3 : 1);
  }
 private:
  double targetrpm = 0;
  TalonSRX* m_Shooter0;
  TalonSRX* m_Shooter1;
  TalonSRX* m_Shooter2;
  TalonSRX* m_Shooter3;
  VictorSPX* m_Indexer;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
