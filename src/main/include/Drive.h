#pragma once

#include "SwerveModule.h"
#include "IOMap.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/AnalogGyro.h"
#include "wpi/array.h"
#include "wpi/math"

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
  
  const units::meters_per_second_t maxSpeed = 1.0_mps;

  /**
   * Processes the odometry and stuff.
   */
  void process();
  
private:
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
   * Resets the encoders on the swerve modules.
   */
  void resetSwerveEncoders();
  
  /**
   * Calibrates the gyro.
   */
  void calibrateGyro();
  
private:
  /**
   * The swerve modules on the robot.
   */
  wpi::array<SwerveModule, 4> swerveModules {
    SwerveModule(SWERVE_FL_DRIVE_MOTOR, SWERVE_FL_ROT_MOTOR, SWERVE_FL_ROT_CAN_CODER),
    SwerveModule(SWERVE_BL_DRIVE_MOTOR, SWERVE_BL_ROT_MOTOR, SWERVE_BL_ROT_CAN_CODER),
    SwerveModule(SWERVE_BR_DRIVE_MOTOR, SWERVE_BR_ROT_MOTOR, SWERVE_BR_ROT_CAN_CODER),
    SwerveModule(SWERVE_FR_DRIVE_MOTOR, SWERVE_FR_ROT_MOTOR, SWERVE_FR_ROT_CAN_CODER),
  };
  
  /**
   * The locations of the swerve modules on the robot.
   *
   * Swerve Module Arrangement:
   *           0 3
   *           1 2
   */
  wpi::array<frc::Translation2d, 4> locations {
    // TODO: Find the actual measurements from center.
    frc::Translation2d(-0.381_m, +0.381_m),
    frc::Translation2d(-0.381_m, -0.381_m),
    frc::Translation2d(+0.381_m, -0.381_m),
    frc::Translation2d(+0.381_m, +0.381_m),
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
  
  frc::AnalogGyro gyro {ANALOG_GYRO};
};
