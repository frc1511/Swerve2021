#pragma once

#include "SwerveModule.h"
#include "IOMap.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/controller/HolonomicDriveController.h"
#include "frc/Timer.h"
#include "adi/ADIS16470_IMU.h"
#include "wpi/array.h"
#include "wpi/math"

// The width between the the left and right swerve modules (not the robot width).
#define ROBOT_WIDTH 0.362_m
// The length between the the back and front swerve modules (not the robot length).
#define ROBOT_LENGTH 0.66_m

/**
 * @brief  Represents the drivetrain of the robot and handles all drive-related functionality.
 */
class Drive {
public:
  Drive();
  ~Drive();
  
  /**
   * @brief  Sets the swerve module states.
   * 
   * @param chassisSpeeds  The speeds of the chassis.
   */
  void setModuleStates(frc::ChassisSpeeds chassisSpeeds);
  
  /**
   * @brief  Sets the drive speeds.
   * 
   * @param xVel  The X velocity of the chassis (-1 to 1).
   * @param yVel  The Y velocity of the chassis (-1 to 1).
   * @param rotVel  The rotational velocity of the chassis (-1 to 1).
   * @param isFieldCentric  Whether to drive using field-centric or robot-centric control.
   */
  void setDrive(double xVel, double yVel, double rotVel, bool isFieldCentric = false);
  
  // The max speed of the robot (depends on weight of course).
  const units::meters_per_second_t maxSpeed = 4_mps;

  /**
   * @brief  Processes odometry and autonomous commands.
   */
  void process();
  
  /**
   * @brief  Returns the rotation of the robot.
   */
  frc::Rotation2d getRotation();

  /**
   * @brief  Updates odometry.
   */
  void updateOdometry();

  /**
   * @brief  Resets the position on the field.
   * 
   * @param pose  The pose to reset to.
   */
  void resetOdometry(frc::Pose2d pose = frc::Pose2d());

  /**
   * @brief  Returns the current pose of the robot.
   */
  frc::Pose2d getPoseMeters();
  
  /**
   * @brief  Resets the angle of the IMU to 0.
   */
  void resetIMU();

  /**
   * @brief  Calibrates the IMU.
   */
  void calibrateIMU();
  
  /**
   * @brief  Begins a command to rotate a specified angle.
   * 
   * @param angle  The angle to rotate.
   */
  void cmdRotate(frc::Rotation2d angle);

  /**
   * @brief  Begins a command to drive a specified distance.
   * 
   * @param x  The amount of meters to move in the x direction.
   * @param y  The amount of meters to move in the x direction.
   * @param angle  The angle to rotate.
   */
  void cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle = frc::Rotation2d());

  /**
   * @brief  Begins a command to drive a specified trajectory.
   * 
   * @param trajectory  The trajectory to follow.
   */
  void cmdTrajectory(frc::Trajectory trajectory);

  /**
   * @brief  Returns whether the last command has finished.
   */
  bool cmdIsFinished();

  /**
   * @brief  Cancels the current command.
   */
  void cmdCancel();

private:

  // The locations of the swerve modules on the robot.
  wpi::array<frc::Translation2d, 4> locations {
    frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front left.
    frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back left.
    frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back right.
    frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front right.
  };

  // The swerve modules on the robot.
  wpi::array<SwerveModule*, 4> swerveModules {
    new SwerveModule(SWERVE_FL_DRIVE_MOTOR, SWERVE_FL_ROT_MOTOR, SWERVE_FL_ROT_CAN_CODER, -1.55),
    new SwerveModule(SWERVE_BL_DRIVE_MOTOR, SWERVE_BL_ROT_MOTOR, SWERVE_BL_ROT_CAN_CODER, -1.59),
    new SwerveModule(SWERVE_BR_DRIVE_MOTOR, SWERVE_BR_ROT_MOTOR, SWERVE_BR_ROT_CAN_CODER, -1.59),
    new SwerveModule(SWERVE_FR_DRIVE_MOTOR, SWERVE_FR_ROT_MOTOR, SWERVE_FR_ROT_CAN_CODER, +1.52),
  };

  // The helper class that converts chassis speeds into swerve module states.
  frc::SwerveDriveKinematics<4> kinematics { locations };
  
  // The odometry class that tracks the position of the robot on the field.
  frc::SwerveDriveOdometry<4> odometry { kinematics, getRotation() };
  
  // The ADIS16470 IMU (3D gyro and accelerometer).
  frc::ADIS16470_IMU imu {};
  
  // The trajectory tracker used to create chassis speeds for a drive command (Input PID values for error correction).
  frc::HolonomicDriveController cmdController { { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0, {} } };

  // Whether a drive command is running.
  bool cmdRunning = false;
  
  // The trajectory for a drive command to follow.
  frc::Trajectory cmdTargetTrajectory;
  
  // A timer to time a drive command.
  frc::Timer cmdTimer {};
};
