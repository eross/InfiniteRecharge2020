/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SparkPIDSource.h"

SparkPIDSource::SparkPIDSource(rev::CANEncoder enc) : enc(enc) 
{

}

SparkPIDSource::~SparkPIDSource()
{
}

double SparkPIDSource::PIDGet()
{
	return enc.GetPosition();
}

void SparkPIDSource::Reset()
{
	enc.SetPosition(0);
}
