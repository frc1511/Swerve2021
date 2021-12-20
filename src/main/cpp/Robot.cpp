#include "Robot.h"

void Robot::RobotInit() {
  for(unsigned i = 0; i < drive.swerveModules.size(); ++i) {
    orchestra.AddInstrument(drive.swerveModules[i]->driveMotor);
  }
}
void Robot::RobotPeriodic() { }

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  orchestra.LoadMusic("home_depo.chrp");
}
void Robot::TeleopPeriodic() {
  controls.process();
  
  if (playHomeDepoSong) {
    orchestra.Play();
  }
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
