#include "SwerveModule.h"

#include "units/math.h"
#include <cmath>

#define DRIVE_ENC_TO_METERS_FACTOR 0.00002226
#define RAD_TO_ENC_FACTOR 10.1859

#define DRIVE_MAX_VOLTAGE 12
#define ROT_MAX_VOLTAGE 12

#define DRIVE_P_VALUE 1000
#define DRIVE_I_VALUE 0
#define DRIVE_D_VALUE 25
#define DRIVE_I_ZONE_VALUE 0
#define DRIVE_FF_VALUE 1023/(DRIVE_MAX_VOLTAGE/DRIVE_ENC_TO_METERS_FACTOR)

#define ROT_P_VALUE 0.4
#define ROT_I_VALUE 0
#define ROT_D_VALUE 0
#define ROT_I_ZONE_VALUE 0
#define ROT_FF_VALUE 0

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderChannel) :
  driveMotorChannel(driveMotorChannel),
  turningMotorChannel(turningMotorChannel),
  canCoderChannel(canCoderChannel),

  driveMotor(driveMotorChannel),
  
  turningMotor(turningMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
  turningRelEncoder(turningMotor.GetEncoder()),
  turningPID(turningMotor.GetPIDController()),
  
  turningAbsSensor(canCoderChannel) {
  
  driveMotor.ConfigFactoryDefault();
  driveMotor.SetNeutralMode(NeutralMode::Brake);
  driveMotor.ConfigVoltageCompSaturation(DRIVE_MAX_VOLTAGE);
  driveMotor.EnableVoltageCompensation(true);
  driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, 0, 0);
  // driveMotor.ConfigSelectedFeedbackCoefficient(   );
  driveMotor.SetInverted(false);
  driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10);
  driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10);
  driveMotor.SetSelectedSensorPosition(0);
  
  driveMotor.Config_kP(0, DRIVE_P_VALUE);
  driveMotor.Config_kI(0, DRIVE_I_VALUE);
  driveMotor.Config_kD(0, DRIVE_D_VALUE);
  driveMotor.Config_IntegralZone(0, DRIVE_I_ZONE_VALUE);
  driveMotor.Config_kF(0, DRIVE_FF_VALUE);
  
  
  turningMotor.RestoreFactoryDefaults();
  turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  turningMotor.EnableVoltageCompensation(ROT_MAX_VOLTAGE);
  turningMotor.SetSmartCurrentLimit(30);
  turningMotor.SetInverted(true);
  turningPID.SetFeedbackDevice(turningRelEncoder);
  
  turningPID.SetP(ROT_P_VALUE, 0);
  turningPID.SetI(ROT_I_VALUE, 0);
  turningPID.SetD(ROT_D_VALUE, 0);
  turningPID.SetIZone(ROT_I_ZONE_VALUE, 0);
  turningPID.SetFF(ROT_FF_VALUE, 0);
  
  turningAbsSensor.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  
  resetEncoders();
}

SwerveModule::~SwerveModule() = default;

void SwerveModule::setState(frc::SwerveModuleState targetState) {
  frc::SwerveModuleState currentState = getState();
  
  // Optimize the target state using the current angle.
  frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
  
  if(optimizedState.speed > 0.0_mps)
    // Rotate the swerve module.
    setTurningMotor(optimizedState.angle.Radians());
  
  // Set the drive motor's velocity.
  setDriveMotor(ControlMode::Velocity, optimizedState.speed.value());
}

frc::SwerveModuleState SwerveModule::getState() {
  return { units::meters_per_second_t(getVelocity()), frc::Rotation2d(getAbsoluteRotation()) };
}

void SwerveModule::resetEncoders() {
  turningRelEncoder.SetPosition(0);
  turningAbsSensor.SetPosition(0);
}

void SwerveModule::setDriveMotor(ControlMode controlMode, double value) {
  driveMotor.Set(controlMode, value);
}

void SwerveModule::setTurningMotor(units::radian_t radians) {
  units::radian_t rotation(radians - getAbsoluteRotation());
  units::radian_t absRotation(units::math::abs(rotation));
  
  // Fix the discontinuity problem.
  // If the value is above π rad or below -π rad...
  if(absRotation.value() > wpi::math::pi)
    // Subtract 2π rad, or add 2π rad depending on the sign.
    rotation = units::radian_t(rotation.value() - (2 * wpi::math::pi) * (std::signbit(rotation.value()) ? -1 : 1));
  
  units::radian_t output(rotation * RAD_TO_ENC_FACTOR);
  output += getRelativeRotation();
  
  // Set PID controller reference.
  turningPID.SetReference(output.value(), rev::ControlType::kPosition);
}

double SwerveModule::getVelocity() {
  return driveMotor.GetSelectedSensorVelocity();
}

units::radian_t SwerveModule::getAbsoluteRotation() {
  return units::radian_t(units::degree_t(turningAbsSensor.GetPosition()));
}

units::radian_t SwerveModule::getRelativeRotation() {
  return units::radian_t(units::degree_t(turningRelEncoder.GetPosition()));
}
