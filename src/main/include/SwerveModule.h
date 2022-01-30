#pragma once

#include "ctre/phoenix.h"
#include "frc/kinematics/SwerveModuleState.h"
// Disable stupid attribute ignored warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#include "rev/CANSparkMax.h"
#pragma GCC diagnostic pop

/**
 * Represents a singular swerve module on the robot.
 */
class SwerveModule {
public:
    SwerveModule(int driveCANID, int turningCANID, int canCoderCANID);
    ~SwerveModule();

    /**
     * Sets the state of the swerve module (Velocity and angle).
     */
    void setState(frc::SwerveModuleState state);

    /**
     * Returns the current state of the swerve module.
     */
    frc::SwerveModuleState getState();

    /**
     * Applies the current rotation of the module as the offset of the
     * CANCoder. Only called when the module was replaced and rotated
     * towards the front of the robot.
     */
    void configOffset();

private:
    /**
     * Sets the speed of the drive motor (-1 to 1).
     */
    void setDriveMotor(double speed);

    /**
     * Returns the current velocity (RPM) of the drive motor.
     */
    double getDriveVelocity();

    /**
     * Sets the angle of the swerve module.
     */
    void setTurningMotor(units::radian_t angle);

    /**
     * Returns the relative rotation of the module (NEO 550 internal encoder).
     */
    double getRelativeRotation();

    /**
     * Returns the absolute rotation of the module (CANCoder).
     */
    units::radian_t getAbsoluteRotation();

    // The drive motor (NEO Brushless motor).
    rev::CANSparkMax driveMotor;
    rev::CANEncoder driveEncoder;
    rev::CANPIDController drivePID;

    // The turning motor (NEO 550).
    rev::CANSparkMax turningMotor;
    rev::CANEncoder turningRelEncoder;
    rev::CANPIDController turningPID;

    // The absolute encoder (CTRE CANCoder).
    ctre::phoenix::sensors::CANCoder turningAbsEncoder;
};
