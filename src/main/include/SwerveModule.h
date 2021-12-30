#pragma once

#include "ctre/phoenix.h"
#include "frc/kinematics/SwerveModuleState.h"
// Disable stupid attribute ignored warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#include "rev/CANSparkMax.h"
#pragma GCC diagnostic pop

/**
 * The SwerveModule class.
 * Represents a single swerve module on the robot.
 */
class SwerveModule {
public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderChannel, double turningOffset);
  ~SwerveModule();
  
  /**
   * Sets the state of the swerve module.
   */
  void setState(frc::SwerveModuleState state);
  
  /**
   * Returns the current state of the swerve module.
   */
  frc::SwerveModuleState getState();

  /**
   * Resets the encoders of the swerve module.
   * TODO Make button to do this.
   */
  void resetEncoders();
  
  /**
   * Sets the mode and mode value of the swerve module's drive motor.
   */
  void setDriveMotor(ControlMode controlMode, double value);
  
  /**
   * Sets the rotation of the swerve module's turning motor.
   */
  void setTurningMotor(units::radian_t radians);
  
  /**
   * Returns the current velocity of the swerve module's drive motor.
   */
  double getVelocity();
  
  /**
   * Returns the current rotation of the CANcoder.
   */
  units::radian_t getAbsoluteRotation();
  
  /**
   * Returns the current rotation of the NEO 550.
   */
  double getRelativeRotation();

  const int driveMotorChannel;
  const int turningMotorChannel;
  const int canCoderChannel;
  
  /**
   * Falcon 500
   * Swerve module drive motor.
   */
  TalonFX driveMotor;
  
  /**
   * NEO 550
   * The swerve module's turning motor.
   */
  rev::CANSparkMax turningMotor;
  rev::CANEncoder turningRelEncoder;
  rev::CANPIDController turningPID;
  
  /**
   * CANcoder
   * The swerve module's magnetic encoder.
   */
  CANCoder turningAbsSensor;
  
  const units::radian_t turningOffset;
};
