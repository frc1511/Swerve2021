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
  /**
   * Constructs a swerve module object. Requires the channels of the drive motor, turning motor, and
   * CANCoder, as well as the turning offset of the CANCoder.
   */
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
   * Sets the swerve module's drive motor to a specified speed.
   */
  void setDriveMotor(double speed);
  
  /**
   * Sets the rotation of the swerve module's turning motor.
   */
  void setTurningMotor(units::radian_t radians);
  
  /**
   * Returns the current velocity of the swerve module's drive motor.
   */
  double getVelocity();
  
  /**
   * Returns the current value of the CANcoder.
   */
  units::radian_t getAbsoluteRotation();
  
  /**
   * Returns the current encoder value of the NEO 550 turning motor.
   */
  double getRelativeRotation();
  
private:
  /**
   * The channels of the motors and encoders.
   */
  const int driveMotorChannel;
  const int turningMotorChannel;
  const int canCoderChannel;

  /**
   * NEO Brushless motor.
   * The drive motor of the swerve module.
   */
  rev::CANSparkMax driveMotor;
  rev::CANEncoder driveEncoder; // TODO Switch to rev::SparkMaxRelativeEncoder for 2022.
  rev::CANPIDController drivePID;
  
  /**
   * NEO 550.
   * The turning motor of the swerve module.
   */
  rev::CANSparkMax turningMotor;
  rev::CANEncoder turningRelEncoder;
  rev::CANPIDController turningPID;
  
  /**
   * CTRE CANcoder.
   * The magnetic encoder of the swerve module (used to read the absolute rotation).
   */
  CANCoder turningAbsSensor;
  
  /**
   * The offset of the CANCoder value.
   */
  const units::radian_t turningOffset;
};
