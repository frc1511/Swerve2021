#include "Robot.h"
#include <cmath>

void Robot::RobotInit() { }
void Robot::RobotPeriodic() { }

void Robot::AutonomousInit() {
  drive.cmdRotate(3.41_rad);
}

void Robot::AutonomousPeriodic() {
  drive.process();
}

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() {
  controls.process();
  drive.process();
  printf("horizontal: %f, vertical: %f\n", limelight.getAngleHorizontal(), limelight.getAngleVertical());
}

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() {
  TeleopInit();
}

void Robot::TestPeriodic() {
  TeleopPeriodic();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
