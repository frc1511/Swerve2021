#include "SwerveModule.h"

#include "units/math.h"
#include <cmath>

// The circumference of each wheel (meters).
#define WHEEL_CIRCUMFERENCE 0.21

// The value of the encoder after 1 rotation.
#define WHEEL_1_ROT_ENC_VAL 1

// The coefficient used to convert the internal drive encoder value to meters (Drive motor).
#define DRIVE_ENC_TO_METERS_FACTOR (WHEEL_CIRCUMFERENCE / WHEEL_1_ROT_ENC_VAL)

// The coeffieient used to convert a radian value to internal encoder value (Turning motor).
#define ROT_RAD_TO_ENC_FACTOR 10.1859

#define DRIVE_MAX_VOLTAGE 12
#define TURNING_MAX_VOLTAGE 12
#define DRIVE_MAX_AMPS 40
#define TURNING_MAX_AMPS 30

// --- PID values ---

// TODO Change PID values for NEOs.
#define DRIVE_P_VALUE 1
#define DRIVE_I_VALUE 0
#define DRIVE_D_VALUE 0
#define DRIVE_I_ZONE_VALUE 0
#define DRIVE_FF_VALUE 0

#define ROT_P_VALUE 0.4
#define ROT_I_VALUE 0
#define ROT_D_VALUE 0
#define ROT_I_ZONE_VALUE 0
#define ROT_FF_VALUE 0
SwerveModule::SwerveModule(int driveCANID, int turningCANID, int canCoderCANID)
  : driveMotor(driveCANID, rev::CANSparkMax::MotorType::kBrushless),
    driveEncoder(driveMotor.GetEncoder()),
    drivePID(driveMotor.GetPIDController()),
    turningMotor(turningCANID, rev::CANSparkMax::MotorType::kBrushless),
    turningRelEncoder(turningMotor.GetEncoder()),
    turningPID(turningMotor.GetPIDController()),
    turningAbsEncoder(canCoderCANID) {
  
    // --- Drive motor config ---
    
    driveMotor.RestoreFactoryDefaults();
    // Brake when idle.
    driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    driveMotor.EnableVoltageCompensation(DRIVE_MAX_VOLTAGE);
    // Limit in amps (Always when using NEO Brushless to avoid damage).
    driveMotor.SetSmartCurrentLimit(DRIVE_MAX_AMPS);
    driveMotor.SetInverted(false);
    // Ramping (0.5 seconds to accelerate from neutral to full throttle).
    driveMotor.SetClosedLoopRampRate(0.5);
    driveMotor.SetOpenLoopRampRate(0.5);
    // Frame period.
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
    driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 10);
    // Apply a coefficient to convert the encoder value to meters.
    driveEncoder.SetPositionConversionFactor(DRIVE_ENC_TO_METERS_FACTOR);
    // PID Values.
    drivePID.SetP(DRIVE_P_VALUE, 0);
    drivePID.SetI(DRIVE_I_VALUE, 0);
    drivePID.SetD(DRIVE_D_VALUE, 0);
    drivePID.SetIZone(DRIVE_I_ZONE_VALUE, 0);
    drivePID.SetFF(DRIVE_FF_VALUE, 0);
  
    // --- Turning motor config ---
    
    turningMotor.RestoreFactoryDefaults();
    turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    turningMotor.EnableVoltageCompensation(TURNING_MAX_VOLTAGE);
    turningMotor.SetSmartCurrentLimit(TURNING_MAX_AMPS);
    turningMotor.SetInverted(true);
    turningPID.SetFeedbackDevice(turningRelEncoder);
    // PID values.
    turningPID.SetP(ROT_P_VALUE, 0);
    turningPID.SetI(ROT_I_VALUE, 0);
    turningPID.SetD(ROT_D_VALUE, 0);
    turningPID.SetIZone(ROT_I_ZONE_VALUE, 0);
    turningPID.SetFF(ROT_FF_VALUE, 0);
    
    // --- CANCoder config ---
    
    turningAbsEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
}

SwerveModule::~SwerveModule() {

}

void SwerveModule::setState(frc::SwerveModuleState targetState) {
    frc::SwerveModuleState currentState = getState();
  
    // Optimize the target state by flipping motor directions and adjusting rotations in order to turn the least amount of distance possible.
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
    
    // Only handle turning when we are actually driving.
    if(units::math::abs(optimizedState.speed) > 0.01_mps) {
        // Rotate the swerve module.
        setTurningMotor(optimizedState.angle.Radians());
    }
  
    // Set the drive motor's velocity.
    setDriveMotor(optimizedState.speed.value());
}

frc::SwerveModuleState SwerveModule::getState() {
    return { units::meters_per_second_t(getDriveVelocity()), frc::Rotation2d(getAbsoluteRotation()) };
}

void SwerveModule::configOffset() {
    turningAbsEncoder.ConfigMagnetOffset(units::degree_t(getAbsoluteRotation()).value());
}

void SwerveModule::setDriveMotor(double speed) {
    driveMotor.Set(speed);
}

double SwerveModule::getDriveVelocity() {
    return driveEncoder.GetVelocity();
}

void SwerveModule::setTurningMotor(units::radian_t angle) {
    // Subtract the absolute rotation from the target rotation.
    units::radian_t rotation(angle - getAbsoluteRotation());
    
    // Fix the discontinuity problem by converting a -2π to 2π value into -π to π value.
    // If the value is above π rad or below -π rad...
    if(units::math::abs(rotation).value() > wpi::math::pi) {
        // Subtract 2π rad, or add 2π rad depending on the sign.
        rotation = units::radian_t(rotation.value() - (2 * wpi::math::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }
    
    // Convert the radian value to internal encoder value.
    double output = rotation.value() * ROT_RAD_TO_ENC_FACTOR;
    
    // Add the current relative rotation.
    output += getRelativeRotation();
    
    // Set PID controller reference.
    // turningPID.SetReference(output, rev::ControlType::kPosition);
    turningPID.SetReference(output, rev::ControlType::kPosition);
}

double SwerveModule::getRelativeRotation() {
    return turningRelEncoder.GetPosition();
}

units::radian_t SwerveModule::getAbsoluteRotation() {
    return units::radian_t(units::degree_t(turningAbsEncoder.GetAbsolutePosition() - 90));
}