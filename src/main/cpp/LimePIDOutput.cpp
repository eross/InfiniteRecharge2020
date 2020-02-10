/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LimePIDOutput.h"

LimePIDOutput::LimePIDOutput()
{

}

LimePIDOutput::~LimePIDOutput() {

}

double LimePIDOutput::GetOutput()
{
    return output;
}

void LimePIDOutput::PIDWrite(double value)
{
    output = value;
}