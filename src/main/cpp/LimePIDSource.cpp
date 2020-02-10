/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LimePIDSource.h"

LimePIDSource::LimePIDSource()
{

}

LimePIDSource::~LimePIDSource()
{
}

double LimePIDSource::PIDGet()
{
	return input;
}

void LimePIDSource::SetInput(double input)
{
    this->input = input;
}
