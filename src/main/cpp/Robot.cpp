#include "Robot.h"

void Robot::RobotInit() {
  
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() {
  controls.process();
}

#include "units/math.h"
#include "units/units.h"
void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() {
  for (int i = 0; i < controls.drive->swerveModules.size(); ++i) {
  printf("[%d] abs: %f, rel: %f, drive: %f\n", i, controls.drive->swerveModules[i]->getAbsoluteRotation().value(), controls.drive->swerveModules[i]->getRelativeRotation(), controls.drive->swerveModules[i]->driveMotor.GetSelectedSensorVelocity());
  
  }
}

void Robot::TestInit() { }
#include <stdio.h>
void Robot::TestPeriodic() { 
  TeleopPeriodic();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
