#include "Robot.h"
#include <cmath>

void Robot::RobotInit() { }
void Robot::RobotPeriodic() { }

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() {
  controls.process();
  printf("%f\n", std::fmod(controls.drive->imu.GetAngle(), 360));
}

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { 
  TeleopPeriodic();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
