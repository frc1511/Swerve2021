#include "Drive.h"

#include "frc/kinematics/SwerveModuleState.h"
#include <cmath>

Drive::Drive() {
  gyro.Reset();
  calibrateGyro();
}

Drive::~Drive() = default;

void Drive::setDrive(frc::ChassisSpeeds chassisSpeeds) {
  // Get target states.
  wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
  
  // Normalize speeds relative to max speed.
  kinematics.NormalizeWheelSpeeds(&moduleStates, maxSpeed);
  
  // Set module states.
  for(unsigned i = 0; i < swerveModules.size(); i++)
    swerveModules.at(i).setState(moduleStates.at(i));
}

void Drive::setDrive(units::velocity::meters_per_second_t xVelMeters,
                     units::velocity::meters_per_second_t yVelMeters,
                     units::degrees_per_second_t degreesPerSecond) {
  // Create chassis speeds struct from parameters, then call the other setDrive function.
  setDrive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVelMeters, yVelMeters, units::radians_per_second_t(degreesPerSecond), getRotation()));
}

frc::Rotation2d Drive::getRotation() {
  double rotation = std::fmod(gyro.GetAngle(), 360);
  
  double absRotation = std::abs(rotation);
  if(absRotation > 180)
    rotation -= 360 * (std::signbit(absRotation) ? -1 : 1);
  
  return frc::Rotation2d(units::degree_t(rotation));
}

void Drive::resetSwerveEncoders() {
  for(SwerveModule& module : swerveModules)
    module.resetEncoders();
}

void Drive::calibrateGyro() {
  gyro.Calibrate();
}
