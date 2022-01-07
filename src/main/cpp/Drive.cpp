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

frc::Rotation2d Drive::getRotation() {
  double absRotation = std::abs(targetRotation);
  if(absRotation > 180)
    targetRotation -= 360 * (std::signbit(absRotation) ? -1 : 1);
  
  return frc::Rotation2d(units::degree_t(targetRotation));
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

void Drive::resetSwerveEncoders() {
  for(SwerveModule* module : swerveModules)
    module->resetEncoders();
}

void Drive::zeroRotation() {
  targetRotation = std::fmod(imu.GetAngle(), 360);
}

void Drive::resetIMU() {
  imu.Reset();
}

void Drive::calibrateIMU() {
  imu.Calibrate();
}
