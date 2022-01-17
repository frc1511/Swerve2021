#include "SwerveModule.h"

#include "units/math.h"
#include <cmath>
#include <stdio.h>

// The circumference of each wheel (meters).
#define WHEEL_CIRCUMFERENCE 0.21

// The value of the encoder after 1 rotation.
#define WHEEL_1_ROT_ENC_VAL 1

// The coefficient used to convert the internal drive encoder value to meters (Drive motor).
#define DRIVE_ENC_TO_METERS_FACTOR (WHEEL_CIRCUMFERENCE / WHEEL_1_ROT_ENC_VAL)

// The coeffieient used to convert a radian value to internal encoder value (Turning motor).
#define ROT_RAD_TO_ENC_FACTOR 10.1859

// Max voltage.
#define MAX_VOLTAGE 12

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

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderChannel, double turningOffset) :
  driveMotorChannel(driveMotorChannel),
  turningMotorChannel(turningMotorChannel),
  canCoderChannel(canCoderChannel),
  driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
  driveEncoder(driveMotor.GetEncoder()),
  drivePID(driveMotor.GetPIDController()),
  turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
  turningRelEncoder(turningMotor.GetEncoder()),
  turningPID(turningMotor.GetPIDController()),
  turningAbsSensor(canCoderChannel),
  turningOffset(turningOffset) {
  
  // --- Drive motor config ---
  
  driveMotor.RestoreFactoryDefaults();
  // Brake when idle.
  driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  driveMotor.EnableVoltageCompensation(MAX_VOLTAGE);
  // Limit in amps (Always when using NEO Brushless to avoid damage).
  driveMotor.SetSmartCurrentLimit(40);
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
  turningMotor.EnableVoltageCompensation(MAX_VOLTAGE);
  turningMotor.SetSmartCurrentLimit(30);
  turningMotor.SetInverted(true);
  turningPID.SetFeedbackDevice(turningRelEncoder);
  // PID values.
  turningPID.SetP(ROT_P_VALUE, 0);
  turningPID.SetI(ROT_I_VALUE, 0);
  turningPID.SetD(ROT_D_VALUE, 0);
  turningPID.SetIZone(ROT_I_ZONE_VALUE, 0);
  turningPID.SetFF(ROT_FF_VALUE, 0);
  
  // --- CANCoder config ---
  
  turningAbsSensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  
  /*
  CANCoderConfiguration config;
  turningAbsSensor.GetAllConfigs(config);
  
  turningAbsSensor.ConfigMagnetOffset(config.magnetOffsetDegrees - turningAbsSensor.GetAbsolutePosition());
  */
}

SwerveModule::~SwerveModule() = default;

void SwerveModule::setState(frc::SwerveModuleState targetState) {
  frc::SwerveModuleState currentState = getState();

  /**
   * Optimize the target state using the current angle of the module.
   * 
   * Optimize() will optimize the target state by flipping motor directions and
   * adjusting rotations in order to turn the least amount of distance possible.
   */
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
  return { units::meters_per_second_t(getVelocity()), frc::Rotation2d(getAbsoluteRotation()) };
}

void SwerveModule::setDriveMotor(double speed) {
  driveMotor.Set(speed);
}

void SwerveModule::setTurningMotor(units::radian_t radians) {
  // Subtract the absolute rotation from the target rotation.
  units::radian_t rotation(radians - getAbsoluteRotation());
  
  // Fix the discontinuity problem.
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
  turningPID.SetReference(output, rev::ControlType::kPosition);
}

double SwerveModule::getVelocity() {
  return driveEncoder.GetVelocity();
}

units::radian_t SwerveModule::getAbsoluteRotation() {
  return units::radian_t(units::degree_t(turningAbsSensor.GetAbsolutePosition())) - turningOffset;
}

double SwerveModule::getRelativeRotation() {
  return turningRelEncoder.GetPosition();
}
