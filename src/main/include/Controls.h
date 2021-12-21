#pragma once
#include "Drive.h"
#include <memory>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <ctre/phoenix/music/Orchestra.h>

class Controls{
public:
    Controls(Drive* drive);
    void process();
    void initMusic();
    
    Drive* drive; 
    frc::Joystick controllerDriver {0};
private:
    bool wasDriveModeToggled = false;
    bool isFieldCentric = false;
    
    bool wasMusicToggled = false;
    bool playMusic = false;
    Orchestra orchestra {};
};