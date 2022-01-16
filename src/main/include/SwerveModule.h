#pragma once

#include "ctre/phoenix.h"
#include "frc/kinematics/SwerveModuleState.h"
// Disable stupid attribute ignored warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#include "rev/CANSparkMax.h"
#pragma GCC diagnostic pop

/**
 * @brief  Represents a singular swerve module on the robot.
 */
class SwerveModule {
public:
  /**
   * @brief  Constructs a swerve module object.
   * 
   * @param driveMotorChannel  The CAN ID of the drive motor (NEO Brushless).
   * @param turningMotorChannel  The CAN ID of the turning motor (NEO 550).
   * @param canCoderChannel  The CAN ID of the absolute encoder (CANCoder).
   * @param turningOffset  The offset of the CANCoder.
   */
  SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderChannel, double turningOffset);
  ~SwerveModule();
  
  /**
   * @brief  Sets the state of the swerve module.
   * 
   * @param state  The state (rotation, velocity, etc.) to move to.
   */
  void setState(frc::SwerveModuleState state);
  
  /**
   * @brief  Retrieves the current state of the swerve module.
   * 
   * @return  The current state of the swerve module.
   */
  frc::SwerveModuleState getState();
  
private:
  
  /**
   * @brief  Sets the drive motor's speed.
   * 
   * @param speed  The speed to set the drive motor to (-1 to 1).
   */
  void setDriveMotor(double speed);
  
  /**
   * @brief  Sets the rotation of the swerve module's turning motor.
   *  
   * @param radians  The angle in radians to turn to.
   */
  void setTurningMotor(units::radian_t radians);
  
  /**
   * @brief  Retrieves the current velocity of the drive motor's encoder.
   * 
   * @return  The velocity.
   */
  double getVelocity();
  
  /**
   * @brief  The current absolute rotation of the module.
   * 
   * @return  The value of the CANcoder.
   */
  units::radian_t getAbsoluteRotation();
  
  /**
   * @brief  Retrieves the current relative rotation of the module.
   * 
   * @return  The value of the NEO 550 encoder.
   */
  double getRelativeRotation();
  
private:
  // The CAN ID of the NEO Brushless.
  const int driveMotorChannel;
  // The CAN ID of the NEO 550.
  const int turningMotorChannel;
  // The CAN ID of the CANCoder.
  const int canCoderChannel;

  // The drive motor (NEO Brushless motor).
  rev::CANSparkMax driveMotor;
  // The encoder of the drive motor.
  rev::CANEncoder driveEncoder;
  // The PID controller of the drive motor.
  rev::CANPIDController drivePID;
  
  // The turning motor (NEO 550).
  rev::CANSparkMax turningMotor;
  // The builtin encoder of the turning motor.
  rev::CANEncoder turningRelEncoder;
  // The PID controller of the turning motor.
  rev::CANPIDController turningPID;
  
  // The absolute encoder (CTRE CANcoder)
  CANCoder turningAbsSensor;
  
  // The offset of the absolute encoder.
  const units::radian_t turningOffset;
};
