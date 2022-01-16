#pragma once
#include "Drive.h"
#include <memory>
#include <frc/Joystick.h>
#include <frc/XboxController.h>

class Controls {
public:
    Controls(Drive* drive);
    void process();
    
    Drive* drive; 
    frc::Joystick controllerDriver {0};
    
private:
    bool wasDriveModeToggled = false;
    bool isFieldCentric = false;
    bool wasSlowModeToggled = false;
    bool slowModeEnabled = false;
};