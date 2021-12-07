#include "Controls.h"
#include <math.h>
#include "rev/CANSparkMax.h"

// Xbox button maps
const int kAButton = 1; // GetRawButton() give bool
const int kBButton = 2; // GetRawButton() give bool
const int kXButton = 3; // GetRawButton() give bool
const int kYButton = 4; // GetRawButton() give bool
const int kLeftBumper = 5; // GetRawButton() give bool
const int kRightBumper = 6; // GetRawButton() give bool
const int kBackButton = 7; // GetRawButton() give bool
const int kStartButton = 8; // GetRawButton() give bool
const int kLeftTrigger = 2; // GetRawAxis() give float
const int kRightTrigger = 3; // GetRawAxis() give float
const int kLeftXAxis = 0; // GetRawAxis() give float
const int kLeftYAxis = 1; // GetRawAxis() give float
const int kRightXAxis = 4; // GetRawAxis() give float
const int kRightYAxis = 5; // GetRawAxis() give float
const int kDPad = 0; // GetPOV() give float

const double kAxisDeadzone = .3;


Controls::Controls(Drive* drive) :drive(drive){

}

#include <stdio.h>
void Controls::process(){
  //puts("Processing controls");
    bool toggleDriveMode = controllerDriver.GetRawButton(kXButton);
    double xAxisVelocity = controllerDriver.GetRawAxis(kLeftXAxis);
    double yAxisVelocity = controllerDriver.GetRawAxis(kLeftYAxis);
    double leftRotationVelocity = controllerDriver.GetRawAxis(kLeftTrigger);
    double rightRotationVelocity = controllerDriver.GetRawAxis(kRightTrigger);
    float slowDriveDirection = controllerDriver.GetPOV(kDPad);
    bool slowLeftVelocity = controllerDriver.GetRawButton(kLeftBumper);
    bool slowRightVelocity = controllerDriver.GetRawButton(kRightBumper);

    if(toggleDriveMode == true) {
        if(wasDriveModeToggled == false) {
            isFieldCentric = !isFieldCentric;
        }
    }

    wasDriveModeToggled = toggleDriveMode;

    double finalXAxis = 0.0;
    double finalYAxis = 0.0;
    double finalRotation = 0.0;

    if(slowDriveDirection == -1){ // DPad not pressed so not in slow mode
        if(abs(xAxisVelocity) > kAxisDeadzone) {
            finalXAxis = xAxisVelocity;
        }
        if(abs(yAxisVelocity) > kAxisDeadzone) {
            finalYAxis = yAxisVelocity;
        }
    }
    else{ // DPad is pressed so is in slow mode
        
         if((slowDriveDirection >=375) || (slowDriveDirection <= 45)){ // dpad up
             finalYAxis = .4;
         }
         if((slowDriveDirection >= 135) && (slowDriveDirection <= 225)){ // dpad down
             finalYAxis = -.4;
         }
         if((slowDriveDirection >= 45) && (slowDriveDirection <= 135)){ // dpad right
             finalXAxis = .4;
         }
         if((slowDriveDirection >= 225) && (slowDriveDirection <= 315)){ // dpad left
             finalXAxis = -.4;
         }
        /*if((slowDriveDirection >=375) || (slowDriveDirection <= 45)){ // dpad up
        //drive swerveModules 0 -> setTurningMotor(0);//fix me!!!
            drive->swerveModules[0]->setTurningMotor(units::radian_t(2));
            puts("01");
        }
        if((slowDriveDirection >= 135) && (slowDriveDirection <= 225)){ // dpad down
            drive->swerveModules[1]->setTurningMotor(units::radian_t(2));
            puts("02");
        }
        if((slowDriveDirection >= 45) && (slowDriveDirection <= 135)){ // dpad right
            drive->swerveModules[2]->setTurningMotor(units::radian_t(2));
            puts("03");
        }
        if((slowDriveDirection >= 225) && (slowDriveDirection <= 315)){ // dpad left
            drive->swerveModules[3]->setTurningMotor(units::radian_t(2));
            puts("04");
        }*/
    }
    if(slowLeftVelocity || slowRightVelocity){ //trigger pressed so slow mode
        if(slowLeftVelocity && slowRightVelocity == false){
            finalRotation = -.25;
        }
        if(slowRightVelocity && slowLeftVelocity == false){
            finalRotation = .25;
        }
    }
    else{ // trigger not pressed so normal mode
        if((leftRotationVelocity > 0) && (rightRotationVelocity == 0)){
            finalRotation = -leftRotationVelocity;
        }
        if((rightRotationVelocity > 0) && (leftRotationVelocity == 0)){
            finalRotation = rightRotationVelocity;
        }
    }

    printf("X: %f, Y: %f\n", finalXAxis, finalYAxis);

    // drive->swerveModules[2]->turningPID.SetReference(1 * 10, rev::ControlType::kPosition);
    
    // drive->swerveModules[0]->setDriveMotor(ControlMode::PercentOutput, 7/11);

    drive->setDrive(units::velocity::meters_per_second_t(finalXAxis), units::velocity::meters_per_second_t(finalYAxis), units::radians_per_second_t(finalRotation));
    // drive->setDrive(units::velocity::meters_per_second_t(1), units::velocity::meters_per_second_t(0), units::radians_per_second_t(0));
}
