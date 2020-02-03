#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() : m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))}
{
    ResetEncoders();

    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
    m_leftEncoder.SetPositionConversionFactor(DriveConst::kencoderConstant);
    m_rightEncoder.SetPositionConversionFactor(DriveConst::kencoderConstant);
    m_leftEncoder.SetVelocityConversionFactor(DriveConst::kencoderConstant * 60);
    m_rightEncoder.SetVelocityConversionFactor(DriveConst::kencoderConstant * 60);
}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic()
{
  std::cout << "distance: " << GetLeftEncoder().GetPosition() << " : " << -GetRightEncoder().GetPosition() << " angle: " << GetHeading() << std::endl;
  frc::Color detectedColor = m_colorSensor.GetColor();
  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
  
  if (matchedColor == kBlueTarget) {
    colorString = "Blue";
  } else if (matchedColor == kRedTarget) {
    colorString = "Red";
  } else if (matchedColor == kGreenTarget) {
    colorString = "Green";
  } else if (matchedColor == kYellowTarget) {
    colorString = "Yellow";
  } else {
    colorString = "Unknown";
  }

  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Color", colorString);
  if(!setDriveReversed)
  {
    m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
      units::meter_t(GetLeftEncoder().GetPosition()),
      units::meter_t(-GetRightEncoder().GetPosition()));
  }
  else
  {
    m_odometry.Update(frc::Rotation2d(units::degree_t(GetHeading())),
      units::meter_t(GetRightEncoder().GetPosition()),
      units::meter_t(-GetLeftEncoder().GetPosition()));
  }
}

void DriveSubsystem::Drive(double forward, double rotate)
{
    m_drive.ArcadeDrive(forward, rotate);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right)
{  
  
  if(!setDriveReversed)
  {
    m_leftMotors.SetVoltage(left);  
    m_rightMotors.SetVoltage(-right);
  }
  else
  {
    m_leftMotors.SetVoltage(-right);  
    m_rightMotors.SetVoltage(left);
  }
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() 
{
  GetLeftEncoder().SetPosition(0);
  GetRightEncoder().SetPosition(0);
}

double DriveSubsystem::GetAverageEncoderDistance()
{   
    return (GetLeftEncoder().GetPosition() - GetRightEncoder().GetPosition()) / 2;
}

rev::CANEncoder DriveSubsystem::GetLeftEncoder() { return m_leftEncoder; }

rev::CANEncoder DriveSubsystem::GetRightEncoder() { return m_rightEncoder; }

void DriveSubsystem::SetMaxOutput(double maxOutput) 
{
    m_drive.SetMaxOutput(maxOutput);
}

double DriveSubsystem::GetHeading() 
{
  return std::remainder(m_gyro.GetAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
}

double DriveSubsystem::GetTurnRate() 
{
  return m_gyro.GetRate() * (kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds()
{
  if(!setDriveReversed)
  {
    return {units::meters_per_second_t(m_leftEncoder.GetVelocity()), units::meters_per_second_t(-m_rightEncoder.GetVelocity())};
  }
  else
  {
    return {units::meters_per_second_t(m_rightEncoder.GetVelocity()), units::meters_per_second_t(-m_leftEncoder.GetVelocity())};
  }
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  ResetEncoders();
  m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));
}