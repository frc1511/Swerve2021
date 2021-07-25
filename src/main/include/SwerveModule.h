#pragma once

#include "ctre/phoenix.h"
#include "rev/CANSparkMax.h"

/**
 * The SwerveModule class.
 * Represents a single swerve module on the robot.
 */
class SwerveModule {
public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel);
  ~SwerveModule();
  
  // Copy constructor necessary due to TalonFX deleted copy constructor.
  SwerveModule(const SwerveModule&);
  
  const int driveMotorChannel;
  const int turningMotorChannel;
  
private:
  TalonFX driveMotor;
  // Should we use 'WPI_TalonFX' instead? IDK.
  
  rev::CANSparkMax turningMotor;
};
