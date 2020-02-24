#include "subsystems/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() : m_odometry{frc::Rotation2d(units::degree_t(GetHeading()))}
{
    ResetEncoders();
    limeOutput = new LimePIDOutput();
    limeSource = new LimePIDSource();
    limePID = new frc::PIDController(0.09, 0.0, 0.0, limeSource, limeOutput);
    limePID->SetOutputRange(-.2, .2);
    limePID->SetSetpoint(-3.3);
    m_leftEncoder.SetPositionConversionFactor(DriveConst::kencoderConstant);
    m_rightEncoder.SetPositionConversionFactor(DriveConst::kencoderConstant);
    m_leftEncoder.SetVelocityConversionFactor(DriveConst::kencoderConstant * 60);
    m_rightEncoder.SetVelocityConversionFactor(DriveConst::kencoderConstant * 60);
}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic()
{
  //std::cout << "distance: " << GetLeftEncoder().GetPosition() << " : " << -GetRightEncoder().GetPosition() << " angle: " << m_gyro.GetAngle() << std::endl;
  //std::cout << GetHeading() << std::endl;
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