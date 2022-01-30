#include "Drive.h"

#include "frc/kinematics/SwerveModuleState.h"
#include <cmath>

Drive::Drive() {
    resetIMU();
    // Configure the calibration time to 4 seconds.
    imu.ConfigCalTime(frc::ADIS16470CalibrationTime::_4s);
    // Set axis for the gryo to take (Z is up and down).
    imu.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
    // Calibrate the IMU.
    calibrateIMU();
}

Drive::~Drive() {
    for (SwerveModule* module : swerveModules) {
        delete module;
    }
}

void Drive::process() {
    // hi nevin
    // hi jeff :D
    // Update the position on the field.
    updateOdometry();

    // Execute the current command.
    executeCommand();
}

frc::Pose2d Drive::getPose() {
    return odometry.GetPose();
}

void Drive::zeroRotation() {
    resetIMU();
}

void Drive::calibrateIMU() {
    imu.Calibrate();
}

void Drive::setupMagneticEncoders() {
    for (SwerveModule* module : swerveModules) {
        module->configOffset();
    }
}

void Drive::manualDrive(double xVel, double yVel, double rotVel) {
    // Take control if drive command is running.
    cmdCancel();
    
    frc::ChassisSpeeds chassisSpeeds;

    // Generate chassis speeds depending on the control mode.
    switch (controlMode) {
        case FIELD_CENTRIC:
            chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(xVel),
                                                                        units::meters_per_second_t(yVel),
                                                                        units::radians_per_second_t(rotVel),
                                                                        getRotation());
            break;
        case ROBOT_CENTRIC:
            chassisSpeeds = {units::meters_per_second_t(xVel),
                             units::meters_per_second_t(yVel),
                             units::radians_per_second_t(rotVel)};
            break;
    }
    
    // Set the modules to turn based on the speeds of the entire chassis.
    setModuleStates(chassisSpeeds);
}

void Drive::setControlMode(ControlMode mode) {
    controlMode = mode;
}

void Drive::makeBrick() {
    for (unsigned i = 0; i < swerveModules.size(); i++) {
        units::degree_t angle;
        // If even index.
        if (i % 2 == 0) {
            angle = -45_deg;
        }
        // If odd index.
        else {
            angle = 45_deg;
        }
        // Turn the swerve module to point towards the center of the robot.
        swerveModules[i]->setState({ 0_mps, angle });
    }
}

void Drive::cmdRotate(frc::Rotation2d angle) {
    // Create a trajectory to rotate the specified angle.
    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        getPose(), {}, { 0_m, 0_m, angle }, AUTO_TRAJECTORY_CONFIG);

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdDrive(units::meter_t x, units::meter_t y, frc::Rotation2d angle) {
    // Create a trajectory that drives the specified distance / turns the specified angle.
    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        getPose(), {}, { x, y, angle }, AUTO_TRAJECTORY_CONFIG);

    // Start a follow trajectory command.
    cmdFollowTrajectory(trajectory);
}

void Drive::cmdFollowTrajectory(frc::Trajectory trajectory) {
    cmd.trajectory = trajectory;
    cmd.timer.Reset();
    cmd.timer.Start();
    cmd.running = true;
}

bool Drive::cmdIsFinished() {
    if (cmd.timer.Get() >= cmd.trajectory.TotalTime().value()) {
        cmd.running = false;
    }
    return cmd.running;
}

void Drive::cmdCancel() {
    cmd.timer.Stop();
    cmd.running = false;
}

void Drive::setModuleStates(frc::ChassisSpeeds chassisSpeeds) {
    // Generate module states using chassis velocities. The magic function of swerve drive.
    wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
    
    // Recalculate wheel speeds relative to the max speed.
    kinematics.NormalizeWheelSpeeds(&moduleStates, DRIVE_MAX_SPEED);
    
    // Set the module states.
    for(unsigned i = 0; i < swerveModules.size(); i++) {
      swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

void Drive::updateOdometry() {
    // Update the position and rotation on the field.
    odometry.Update(getRotation(),
        swerveModules.at(0)->getState(),
        swerveModules.at(1)->getState(),
        swerveModules.at(2)->getState(),
        swerveModules.at(3)->getState());
}

void Drive::resetOdometry(frc::Pose2d pose) {
    // Reset the position on the field.
    odometry.ResetPosition(pose, getRotation());
}

void Drive::resetIMU() {
    imu.Reset();
}

frc::Rotation2d Drive::getRotation() {
    units::radian_t rotation = units::degree_t(fmod(imu.GetAngle(), 360));

    // Convert -2π to 2π value into -π to π value.
    if(units::math::abs(rotation).value() > wpi::math::pi) {
        rotation = units::radian_t(rotation.value() - (2 * wpi::math::pi) * (std::signbit(rotation.value()) ? -1 : 1));
    }
    
    return frc::Rotation2d(units::degree_t(rotation));
}

void Drive::executeCommand() {
    // Only execute a command if the command is running.
    if (cmd.running) {
        // Check if the timer timer has passed the total time of the trajectory.
        if (cmdIsFinished()) {
            cmdCancel();
        }
        // Drive!!
        else {
            // Get the current time of the trajectory.
            units::second_t currentTime = units::second_t(cmd.timer.Get());
            // Sample the desired state of the trajectory at this point in time.
            frc::Trajectory::State desiredState = cmd.trajectory.Sample(currentTime);
            // Calculate chassis speeds required in order to reach the desired state.
            frc::ChassisSpeeds targetChassisSpeeds = cmdController.Calculate(
                getPose(), desiredState, cmd.trajectory.States().back().pose.Rotation());
            // Finally drive!
            setModuleStates(targetChassisSpeeds);
        }
    }
    //hi ishan
    //hi jeff
    //hi trevor
}