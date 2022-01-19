#include "Limelight.h"

#define LIMELIGHT_ANGLE 28 // degrees

Limelight::Limelight() {
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
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

Limelight::~Limelight() {
  
}