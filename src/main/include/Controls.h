#pragma once

#include "Drive.h"
#include <frc/PS4Controller.h>

class Controls {
public:
    Controls(Drive* drive);
    void process();
    
    Drive* drive;

    frc::PS4Controller driverController {0};
    frc::PS4Controller auxController {1};
    
private:
    bool wasDriveModeToggled = false;
    bool isFieldCentric = false;
    bool wasSlowModeToggled = false;
    bool slowModeEnabled = false;
};