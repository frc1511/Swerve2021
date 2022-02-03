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
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include "frc/controller/HolonomicDriveController.h"
#include "frc/Timer.h"
#include "adi/ADIS16470_IMU.h"
#include "wpi/array.h"
#include "wpi/math"
#include <wpi/FileSystem.h>
#include <fstream>

#define DRIVE_MAX_SPEED 4_mps
#define DRIVE_MAX_ANGULAR_SPEED 3.14_rad_per_s

#define AUTO_MAX_SPEED 1_mps
#define AUTO_MAX_ANGULAR_SPEED 3.14_rad_per_s
#define AUTO_MAX_ACCELERATION 0.4_mps_sq
#define AUTO_MAX_ANGULAR_ACCELERATION (3.14_rad_per_s / 1_s)

#define AUTO_TRAJECTORY_CONFIG (frc::TrajectoryConfig(AUTO_MAX_SPEED, AUTO_MAX_ACCELERATION))
#define AUTO_TRAJECTORY_ANGULAR_CONSTRAINTS (frc::TrapezoidProfile<units::radians>::Constraints(AUTO_MAX_ANGULAR_SPEED, AUTO_MAX_ANGULAR_ACCELERATION))

// The width between the the left and right swerve modules (not the robot width).
#define ROBOT_WIDTH 0.362_m
// The length between the the back and front swerve modules (not the robot length).
#define ROBOT_LENGTH 0.66_m

/**
 * Represents the drivetrain of the robot and handles all drive-related
 * functionality.
 */
class Drive {
public:
    Drive();
    ~Drive();
  
    void reset();
    void process();
        
    /**
     * Returns the current pose (X, Y position and rotation).
     */
    frc::Pose2d getPose();

    /**
     * Resets the forward direction of the robot during field-centric control.
     */
    void zeroRotation();

    /**
     * Calibrates the IMU (Should be done only once at a time when the robot is
     * not moving).
     */
    void calibrateIMU();

    /**
     * Applies the current rotation of the swerve modules as the offset of the
     * magnetic encoders. *** IMPORTANT *** Should only be called after
     * replacing a swerve module and when all the swerve modules are rotated
     * towards the front of the robot!
     */
    void configMagneticEncoders();
    
    /**
     * Manually control the robot using percentages of the max output velocities.
     * (dependant on control type).
     */
    void manualDrive(double xPct, double yPct, double rotPct);

    enum ControlMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    };

    /**
     * Sets the control mode of the robot (relative to the robot or relative to
     * the field).
     */
    void setControlMode(ControlMode mode);

    /**
     * Rotates all the swerve modules towards the center of the robot in order
     * to reduce pushing by other robots (aka making the robot a brick).
     */
    void makeBrick();

    // --- Commands ---

    /**
     * Commands are used primarily during the autonomous period in order to
     * instruct the drivetrain to execute an action, such as rotating, driving,
     * or following a trajectory. Commands are mutually exclusive, meaning that
     * when the drive is instructed to execute a command, it will abort any
     * other ongoing commands and immediately begin the new one.
     */

    /**
     * Begins a command to rotate a specified angle.
     */
    void cmdRotate(frc::Rotation2d angle);

    /**
     * Begins a command to drive a specified distance and rotate a specified
     * angle.
     */
    void cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle = frc::Rotation2d());
    
    /**
     * Begins a command to follow a specified trajectory.
     */
    void cmdFollowTrajectory(frc::Trajectory trajectory);

    /**
     * Begins a command to follow a pathweaver trajecectory from a json file.
     */
    void cmdFollowPathweaverTrajectory(std::string path);

    /**
     * Returns whether the last command has finished.
     */
    bool cmdIsFinished();

    /**
     * Cancels the current command.
     */
    void cmdCancel();

private:
    /**
     * Generates and sends states to the swerve modules from chassis speeds.
     */
    void setModuleStates(frc::ChassisSpeeds chassisSpeeds);

    /**
     * Updates the position and rotation on the field.
     */
    void updateOdometry();

    /**
     * Resets the position and rotation on the field.
     */
    void resetOdometry(frc::Pose2d pose = frc::Pose2d());

    /**
     * Resets the IMU to 0.
     */
    void resetIMU();
    
    /**
     * Returns the rotation of the robot.
     */
    frc::Rotation2d getRotation();

    /**
     * Executes the current command.
     */
    void executeCommand();

    /**
     * Reads the magnetic encoder offsets file.
     */
    bool readOffsetsFile();

    /**
     * Writes the current magnetic encoder offsets to the file.
     */
    void writeOffsetsFile();

    /**
     * Applies the current offsets to the swerve modules.
     */
    void applyOffsets();

    // The control mode of the robot.
    ControlMode controlMode = FIELD_CENTRIC;

    // The locations of the swerve modules on the robot.
    wpi::array<frc::Translation2d, 4> locations {
        frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front left.
        frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back left.
        frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back right.
        frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front right.
    };

    // The swerve modules on the robot.
    wpi::array<SwerveModule*, 4> swerveModules {
      new SwerveModule(SWERVE_FL_DRIVE_MOTOR, SWERVE_FL_ROT_MOTOR, SWERVE_FL_ROT_CAN_CODER),
      new SwerveModule(SWERVE_BL_DRIVE_MOTOR, SWERVE_BL_ROT_MOTOR, SWERVE_BL_ROT_CAN_CODER),
      new SwerveModule(SWERVE_BR_DRIVE_MOTOR, SWERVE_BR_ROT_MOTOR, SWERVE_BR_ROT_CAN_CODER),
      new SwerveModule(SWERVE_FR_DRIVE_MOTOR, SWERVE_FR_ROT_MOTOR, SWERVE_FR_ROT_CAN_CODER),
    };

    // The offsets of the swerve modules.
    wpi::array<units::radian_t, 4> offsets { 0_rad, 0_rad, 0_rad, 0_rad };

    // The helper class that converts chassis speeds into swerve module states.
    frc::SwerveDriveKinematics<4> kinematics { locations };

    // The odometry class that tracks the position of the robot on the field.
    frc::SwerveDriveOdometry<4> odometry { kinematics, getRotation() };

    // The ADIS16470 IMU (3D gyro and accelerometer) in the SPI port on the
    // roborio.
    frc::ADIS16470_IMU imu {};

    struct SwerveCommand {
        frc::Trajectory trajectory;
        frc::Timer timer;
        bool running = false;
    };

    SwerveCommand cmd {};

    frc::HolonomicDriveController cmdController { { 1, 0, 0 }, { 1, 0, 0 }, { 1, 0, 0, AUTO_TRAJECTORY_ANGULAR_CONSTRAINTS } };
};