/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"

class LimePIDSource : public frc::PIDSource
{
  double input;
 private:
 public:
  LimePIDSource();
  void SetInput(double input);
	virtual ~LimePIDSource();
	virtual double PIDGet();
};