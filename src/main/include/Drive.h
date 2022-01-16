#pragma once

#include "SwerveModule.h"
#include "IOMap.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "adi/ADIS16470_IMU.h"
#include "wpi/array.h"
#include "wpi/math"

#define ROBOT_WIDTH 0.362_m
#define ROBOT_LENGTH 0.66_m

/**
 * The Drive class.
 * Handles the driving of the robot.
 */
class Drive {
public:
  Drive();
  ~Drive();
  
  /**
   * Sets the swerve module states using chassis speeds.
   */
  void setDrive(frc::ChassisSpeeds chassisSpeeds);
  
  void setDrive(units::velocity::meters_per_second_t xVelMeters,
                units::velocity::meters_per_second_t yVelMeters,
                units::degrees_per_second_t degreesPerSecond,
                bool isFieldCentric = false);
  
  const units::meters_per_second_t maxSpeed = 4_mps;

  /**
   * Processes the odometry and stuff.
   */
  void process();
  

  /**
   * Returns the rotation of the robot.
   */
  frc::Rotation2d getRotation();

  /**
   * Updates the odometry.
   */
  void updateOdometry();

  /**
   * Resets the odometry.
   */
  void resetOdometry(frc::Pose2d pose);

  /**
   * Returns the Pose2d.
   */
  frc::Pose2d getPoseMeters();
  
  /**
   * Resets the angle of the IMU.
   */
  void resetIMU();

  /**
   * Calibrates the IMU.
   */
  void calibrateIMU();

  /**
   * The locations of the swerve modules on the robot.
   *
   * Swerve Module Arrangement:
   *           0 3
   *           1 2
   */
  wpi::array<frc::Translation2d, 4> locations {
    frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2),
    frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2),
    frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2),
    frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2),
  };

  /**
   * The swerve modules on the robot.
   */
  wpi::array<SwerveModule*, 4> swerveModules {
    new SwerveModule(SWERVE_FL_DRIVE_MOTOR, SWERVE_FL_ROT_MOTOR, SWERVE_FL_ROT_CAN_CODER, -0.09),
    new SwerveModule(SWERVE_BL_DRIVE_MOTOR, SWERVE_BL_ROT_MOTOR, SWERVE_BL_ROT_CAN_CODER, +0.29),
    new SwerveModule(SWERVE_BR_DRIVE_MOTOR, SWERVE_BR_ROT_MOTOR, SWERVE_BR_ROT_CAN_CODER, +1.25),
    new SwerveModule(SWERVE_FR_DRIVE_MOTOR, SWERVE_FR_ROT_MOTOR, SWERVE_FR_ROT_CAN_CODER, +1.35),
  };

  /**
   * The helper class that converts chassis speeds into
   * swerve module states.
   */
  frc::SwerveDriveKinematics<4> kinematics { locations };

  /**
   * The odometry class that handles field-centric stuff.
   */
  frc::SwerveDriveOdometry<4> odometry { kinematics, getRotation() };
  
  /**
   * The ADIS16470 IMU
   */
  frc::ADIS16470_IMU imu {};
};
