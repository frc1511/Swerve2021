#pragma once

#include "Drive.h"
#include "Controls.h"
#include <frc/TimedRobot.h>
#include <ctre/phoenix/music/Orchestra.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  
  void RobotPeriodic() override;
  
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  
  void TeleopInit() override;
  void TeleopPeriodic() override;
  
  void DisabledInit() override;
  void DisabledPeriodic() override;
  
  void TestInit() override;
  void TestPeriodic() override;
  
private:
  bool playHomeDepoSong = false;
  Orchestra orchestra {};
  
  Drive drive {};
  Controls controls {&drive, &playHomeDepoSong};
};
