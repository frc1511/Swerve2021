#pragma once
#include "Drive.h"
#include <memory>
#include <frc/Joystick.h>
#include "frc/XboxController.h"

class Controls{
    public:
    Controls(Drive* drive);
    void process();
    
    private:
    std::unique_ptr<Drive> drive; 
    frc::Joystick controllerDriver{0};

    bool isFieldCentric = false;
    bool wasDriveModeToggled = false;

};