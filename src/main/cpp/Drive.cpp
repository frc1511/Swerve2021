#include "Drive.h"

#include "frc/kinematics/SwerveModuleState.h"
#include <cmath>
#include <stdio.h>

Drive::Drive() {
  resetIMU();
  // Configure the calibration time to 4 seconds.
  imu.ConfigCalTime(frc::ADIS16470CalibrationTime::_4s);
  // Set axis for the gryo to take (Z is up and down).
  imu.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
  // Calibrate the IMU.
  calibrateIMU();
}

Drive::~Drive() {
  for(SwerveModule* module : swerveModules)
    delete module;
}

void Drive::setModuleStates(frc::ChassisSpeeds chassisSpeeds) {
  /**
   * Generate module states using chassis velocities.
   * The magic function of swerve drive.
   */
  wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
  
  // Recalculate wheel speeds relative to the max speed.
  kinematics.NormalizeWheelSpeeds(&moduleStates, maxSpeed);
  
  // Set the module states.
  for(unsigned i = 0; i < swerveModules.size(); i++) {
    swerveModules.at(i)->setState(moduleStates.at(i));
  }
}

void Drive::setDrive(double xVel, double yVel, double rotVel, bool isFieldCentric) {
  // Take control if drive command is running.
  cmdCancel();
  
  if(isFieldCentric) {
    // Generate relative chassis speeds from velocities based on the robot's current rotation on the field.
    setModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(xVel), units::meters_per_second_t(yVel), units::radians_per_second_t(rotVel), getRotation()));
  }
  else {
    // Directly use velocities for robot-centric control.
    setModuleStates({ units::meters_per_second_t(xVel), units::meters_per_second_t(yVel), units::radians_per_second_t(rotVel) });
  }
}

void Drive::process() {
  updateOdometry();
  
  // If a drive command is executing.
  if (cmd.has_value()) {
    if (!cmd.value().IsFinished()) {
      cmd.value().Execute();
    }
  }
}

frc::Rotation2d Drive::getRotation() {
  double angle = std::fmod(imu.GetAngle(), 360);
  
  double rotation = angle;
  
  // Convert -360 to 360 value into -180 to 180 value.
  if(abs(rotation) > 180) {
    rotation -= (360 * (std::signbit(rotation) ? -1 : 1));
  }
  
  return frc::Rotation2d(units::degree_t(rotation));
}

void Drive::updateOdometry() {
  // Update the position and rotation on the field.
  odometry.Update(getRotation(),
    swerveModules.at(0)->getState(),
    swerveModules.at(1)->getState(),
    swerveModules.at(2)->getState(),
    swerveModules.at(3)->getState());
}

void Drive::resetOdometry(frc::Pose2d pose) {
  // Reset the position on the field.
  odometry.ResetPosition(pose, getRotation());
}

frc::Pose2d Drive::getPoseMeters() {
  return odometry.GetPose();
}

void Drive::resetIMU() {
  imu.Reset();
}

void Drive::calibrateIMU() {
  imu.Calibrate();
}

void Drive::cmdRotate(frc::Rotation2d angle) {
  cmdTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(getPoseMeters(), {}, { 0_m, 0_m, angle }, getTrajectoryConfig()));
}

void Drive::cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle) {
  cmdTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(getPoseMeters(), {}, { x, y, angle }, getTrajectoryConfig()));
}

void Drive::cmdTrajectory(frc::Trajectory trajectory) {
  // Cancel a command if running.
  cmdCancel();

  cmd = frc2::SwerveControllerCommand<4>(
    trajectory,
    std::function<frc::Pose2d()>(std::bind(&Drive::getPoseMeters, this)),
    kinematics,
    frc2::PIDController(1, 0, 0),
    frc2::PIDController(1, 0, 0),
    frc::ProfiledPIDController<units::radians>(1, 0, 0, {}),
    std::function<void(std::array<frc::SwerveModuleState, 4>)>([=](std::array<frc::SwerveModuleState, 4> states){
      for(unsigned i = 0; i < this->swerveModules.size(); i++) {
        this->swerveModules.at(i)->setState(states.at(i));
      }
    })
  );

  cmd.value().Initialize();
}

bool Drive::cmdIsFinished() {
  if (!cmd.has_value()) return true;
  return cmd.value().IsFinished();
}

void Drive::cmdCancel() {
  if (cmd.has_value()) {
    cmd.value().Cancel();
  }
}

frc::TrajectoryConfig Drive::getTrajectoryConfig() {
  return frc::TrajectoryConfig(0.3_mps, 0.1_mps_sq);
}