/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/PIDSource.h>
#include "rev/CANSparkMax.h"

class SparkPIDSource : public frc::PIDSource
{

 private:
  rev::CANEncoder enc;
 public:
  SparkPIDSource(rev::CANEncoder enc);
  void Reset();
	virtual ~SparkPIDSource();
	virtual double PIDGet();
};
