#pragma once
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <memory>

class Limelight {
public:
  Limelight();
  ~Limelight();

  double getAngleHorizontal();
  double getAngleVertical();
  
private:
  std::shared_ptr<NetworkTable> table;
};