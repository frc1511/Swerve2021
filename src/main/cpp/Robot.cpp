#include "Robot.h"

void Robot::RobotInit() { }
void Robot::RobotPeriodic() { }

void Robot::AutonomousInit() {
  drive.reset();
  // drive.cmdRotate(90_deg);/
  drive.cmdDrive(5_m, 5_m, 0_deg);
  // drive.cmdDrive(0_m, 0_m, 90_deg);
}

void Robot::AutonomousPeriodic() {
  // printf("periodic...\n");
  drive.process();
}

void Robot::TeleopInit() {
  drive.reset();
}

void Robot::TeleopPeriodic() {
  controls.process();
  drive.process();
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
