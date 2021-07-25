#include "SwerveModule.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel) :
  driveMotor(driveMotorChannel),
  turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
  driveMotorChannel(driveMotorChannel),
  turningMotorChannel(turningMotorChannel) {
  
  
}

SwerveModule::~SwerveModule() { }

SwerveModule::SwerveModule(const SwerveModule& swerveModule) :
  // Call the regular constructor and only use the previous motor channels.
  SwerveModule(swerveModule.driveMotorChannel, swerveModule.turningMotorChannel) { }