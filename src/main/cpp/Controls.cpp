#include "Controls.h"
#include <math.h>
#include "rev/CANSparkMax.h"

#define AXIS_DEADZONE .05

Controls::Controls(Drive* drive) : drive(drive) { }

void Controls::process() {
    bool zeroRotation    = driverController.GetTriangleButton();
    bool toggleSlowMode  = driverController.GetCircleButton();
    bool brickDrive      = driverController.GetCrossButton();
    bool toggleDriveMode = driverController.GetSquareButton();

    double xVelocity     = driverController.GetLeftX();
    double yVelocity     = driverController.GetLeftY();
    double rotVelocity   = driverController.GetRightX();

    if(zeroRotation) {
        drive->zeroRotation();
    }

    if(toggleDriveMode) {
        if(!wasDriveModeToggled) {
            isFieldCentric = !isFieldCentric;
            drive->setControlMode(isFieldCentric ? Drive::FIELD_CENTRIC : Drive::ROBOT_CENTRIC);
        }
    }
    wasDriveModeToggled = toggleDriveMode;

    if(toggleSlowMode) {
        if(!wasSlowModeToggled) {
            slowModeEnabled = !slowModeEnabled;
        }
    }
    wasSlowModeToggled = toggleSlowMode;

    if(brickDrive) {
        drive->makeBrick();
    }

    double finalXVelocity = 0.0;
    double finalYVelocity = 0.0;
    double finalRotVelocity = 0.0;

    if(slowModeEnabled) {
        if(abs(xAxisVelocity) > AXIS_DEADZONE) {
            finalXVelocity = .2 * xVelocity;
        }
        if(abs(yAxisVelocity) > AXIS_DEADZONE) {
            finalYVelocity = .2 * yVelocity;
        }
        if(abs(rotVelocity) > AXIS_DEADZONE) {
            finalRotVelocity = .2 * rotVelocity;
        }
    }
    else {
        if(abs(xAxisVelocity) > AXIS_DEADZONE) {
            finalXVelocity = xVelocity;
        }
        if(abs(yAxisVelocity) > AXIS_DEADZONE) {
            finalYVelocity = yVelocity;
        }
        if(abs(rotVelocity) > AXIS_DEADZONE) {
            finalRotVelocity = rotVelocity;
        }
    }

    drive->manualDrive(-finalXVelocity, finalYVelocity, finalRotVelocity);
}