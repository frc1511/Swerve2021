#include "Robot.h"
#include <cmath>

void Robot::RobotInit() { }
void Robot::RobotPeriodic() { }

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() {
  controls.process();
  printf("%f\n", (double)fmod(controls.drive->gyro.GetAngle(), 360));
  // printf("%f\n", (double)controls.drive->gyro.GetAngle());
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
