#pragma once
#include "Drive.h"
#include <memory>
#include <frc/Joystick.h>
#include "frc/XboxController.h"

class Controls{
public:
    Controls(Drive* drive);
    void process();
    
    std::unique_ptr<Drive> drive; 
    frc::Joystick controllerDriver {0};
private:
    bool isFieldCentric = false;
};