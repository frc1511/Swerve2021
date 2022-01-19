#include "Limelight.h"

#define LIMELIGHT_ANGLE 28 // degrees

Limelight::Limelight() {
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

Limelight::~Limelight() {
  
}

bool Limelight::hasTarget() {
  return table->GetNumber("tv", 0.0);
}

double Limelight::getAngleHorizontal() {
  return table->GetNumber("tx", 0.0);
}

double Limelight::getAngleVertical() {
  return table->GetNumber("ty", 0.0) ;
}

void Limelight::setLEDMode(LEDMode mode) {
  table->PutNumber("ledMode", (int)mode);
}

Limelight::LEDMode Limelight::getLEDMode() {
  return (LEDMode)(int)table->GetNumber("ledMode", 0.0);
}

void Limelight::setCameraMode(CameraMode mode) {
  table->PutNumber("camMode", (int)mode);
}

Limelight::CameraMode Limelight::getCameraMode() {
  return (CameraMode)(int)table->GetNumber("camMode", 0.0);
}