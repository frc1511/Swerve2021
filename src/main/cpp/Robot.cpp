#include "Robot.h"
#include <cmath>

void Robot::RobotInit() { }
void Robot::RobotPeriodic() { }

void Robot::AutonomousInit() {
  printf("auto???\n");
  drive.cmdRotate(90_deg);
}

void Robot::AutonomousPeriodic() {
  // printf("periodic...\n");
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
