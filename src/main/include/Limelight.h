#pragma once
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <memory>

/**
 * @brief  Represents the limelight optical sensor on the robot.
 */
class Limelight {
public:
  Limelight();
  ~Limelight();

  /**
   * @brief  Returns whether the limelight has any valid targets.
  */
  bool hasTarget();

  /**
   * @brief  Returns the horizontal angle to target in degrees.
   */
  double getAngleHorizontal();

  /**
   * @brief  Returns the vertical angle to target in degrees.
   */
  double getAngleVertical();

  enum LEDMode {
    Default = 0,
    Off = 1,
    Blink = 2,
    On = 3,
  };

  /**
   * @brief  Sets the LED mode.
   * 
   * @param mode  The mode to switch to.
   */
  void setLEDMode(LEDMode mode);

  /**
   * @brief  Returns the current LED mode.
   */
  LEDMode getLEDMode();

  enum CameraMode {
    VisionProcess = 0,
    DriverCamera = 1,
  };

  /**
   * @brief  Sets the camera mode.
   * 
   * @param mode  The mode to switch to.
   */
  void setCameraMode(CameraMode mode);
  
  /**
   * @brief  Returns the current camera mode.
   */
  CameraMode getCameraMode();
  
private:
  std::shared_ptr<NetworkTable> table;
};