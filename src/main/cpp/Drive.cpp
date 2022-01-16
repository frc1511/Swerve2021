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

void Drive::setDrive(frc::ChassisSpeeds chassisSpeeds) {
  // Get target states.
  wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
  
  // Normalize speeds relative to max speed.
  kinematics.NormalizeWheelSpeeds(&moduleStates, maxSpeed);
  // Set module states.
  for(unsigned i = 0; i < swerveModules.size(); i++) {
    swerveModules.at(i)->setState(moduleStates.at(i));
  }
}

void Drive::setDrive(units::velocity::meters_per_second_t xVelMeters,
                     units::velocity::meters_per_second_t yVelMeters,
                     units::degrees_per_second_t degreesPerSecond,
                     bool isFieldCentric) {
  if(isFieldCentric)
    // Create relative chassis speeds struct from parameters, then call the other setDrive function.
    setDrive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVelMeters, yVelMeters, units::radians_per_second_t(degreesPerSecond), getRotation()));
  else
    // Create chassis speeds struct from parameters, then call the other setDrive function.
    setDrive({ xVelMeters, yVelMeters, units::radians_per_second_t(degreesPerSecond) });
}

void Drive::process() {
  updateOdometry();
}

#define FROM_360_TO_PLUS_MINUS_180(rotation) \
  (abs(rotation) > 180 ? rotation - 360 * (std::signbit(rotation) ? -1 : 1) : rotation)

frc::Rotation2d Drive::getRotation() {
  double angle = std::fmod(imu.GetAngle(), 360);
  
  // The rotation is the change of the imu's angle since it's last reset.
  double rotation = FROM_360_TO_PLUS_MINUS_180(angle);
  
  return frc::Rotation2d(units::degree_t(rotation));
}

void Drive::updateOdometry() {
  odometry.Update(getRotation(), swerveModules.at(0)->getState(), swerveModules.at(1)->getState(), swerveModules.at(2)->getState(), swerveModules.at(3)->getState());
}

void Drive::resetOdometry(frc::Pose2d pose) {
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
