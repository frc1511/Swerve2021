#pragma once

#include "SwerveModule.h"
#include "IOMap.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/AnalogGyro.h"
#include "wpi/array.h"

/**
 * The Drive class.
 * Handles the driving of the robot.
 */
class Drive {
public:
  Drive();
  ~Drive();
  
  void drive();
  
private:
  /**
   * Module Arrangement:
   *    0 3
   *    1 2
   */
  wpi::array<SwerveModule, 4> swerveModules {
    SwerveModule(SWERVE_FL_DRIVE, SWERVE_FL_ROT),
    SwerveModule(SWERVE_BL_DRIVE, SWERVE_BL_ROT),
    SwerveModule(SWERVE_BR_DRIVE, SWERVE_BR_ROT),
    SwerveModule(SWERVE_FR_DRIVE, SWERVE_FR_ROT),
  };
  
  wpi::array<frc::Translation2d, 4> locations {
    // TODO: Find the actual measurements from center.
    frc::Translation2d(-0.381_m, +0.381_m),
    frc::Translation2d(-0.381_m, -0.381_m),
    frc::Translation2d(+0.381_m, -0.381_m),
    frc::Translation2d(+0.381_m, +0.381_m),
  };
  
  frc::SwerveDriveKinematics<4> kinematics { locations };
  
  frc::AnalogGyro gyro {0};
};
