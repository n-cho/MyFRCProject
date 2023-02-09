// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Created by Nathan Cho on 2023-02-04.
//

#include <cmath>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "ctre/Phoenix.h"
#include "frc/XboxController.h"
#include "AHRS.h"

#include <Robot.h>
#include <DeviceManager.h>

TalonFX FrontLeftDrive{kFrontLeftDriveID};
TalonFX FrontLeftSteering{kFrontLeftSteeringID};
TalonFX FrontRightDrive{kFrontRightDriveID};
TalonFX FrontRightSteering{kFrontRightSteeringID};
TalonFX BackLeftDrive{kBackLeftDriveID};
TalonFX BackLeftSteering{kBackLeftSteeringID};
TalonFX BackRightDrive{kBackRightDriveID};
TalonFX BackRightSteering{kBackRightSteeringID};

CANCoder BackLeftCANCoder{kBackLeftCANCoderID};
CANCoder BackRightCANCoder{kBackRightCANCoderID};
CANCoder FrontLeftCANCoder{kFrontLeftCANCoderID};
CANCoder FrontRightCANCoder{kFrontRightCANCoderID};

AHRS navX{frc::SPI::kMXP};

frc::XboxController Controller{kControllerID};

// Locations for the swerve drive modules relative to the robot center.
// Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
const frc::Translation2d kFrontLeftLocationFromCenter{11.625_in, 7.25_in};
const frc::Translation2d kFrontRightLocationFromCenter{11.625_in, -7.25_in};
const frc::Translation2d kBackLeftLocationFromCenter{-11.625_in, 7.25_in};
const frc::Translation2d kBackRightLocationFromCenter{-11.625_in, -7.25_in};

// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> kinematics{
  kFrontLeftLocationFromCenter, 
  kFrontRightLocationFromCenter,
  kBackLeftLocationFromCenter, 
  kBackRightLocationFromCenter
};

units::length::inch_t falconFXToInches(double selectedSensorPosition) {
  return units::length::inch_t{(selectedSensorPosition * 4 * M_PI) / 180};
}

frc::SwerveModulePosition frontLeftModulePosition{falconFXToInches(FrontLeftDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FrontLeftCANCoder.GetPosition()})};
frc::SwerveModulePosition frontRightModulePosition{falconFXToInches(FrontRightDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FrontRightCANCoder.GetPosition()})};
frc::SwerveModulePosition backLeftModulePosition{falconFXToInches(BackLeftDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BackLeftCANCoder.GetPosition()})};
frc::SwerveModulePosition backRightModulePosition{falconFXToInches(BackRightDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BackRightCANCoder.GetPosition()})};

// Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.
frc::SwerveDriveOdometry<4> odometry{
  kinematics,
  navX.GetRotation2d(),
  {
    frc::SwerveModulePosition{falconFXToInches(FrontLeftDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FrontLeftCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{falconFXToInches(FrontRightDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FrontRightCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{falconFXToInches(BackLeftDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BackLeftCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{falconFXToInches(BackRightDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BackRightCANCoder.GetPosition()})}
  },
  frc::Pose2d{0_m, 0_m, 0_deg}
  };

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  // Get the rotation of the robot from the gyro.
  frc::Rotation2d gyroAngle = navX.GetRotation2d();

  // Update the pose
  auto pose = odometry.Update(gyroAngle,
  {
    frc::SwerveModulePosition{falconFXToInches(FrontLeftDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FrontLeftCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{falconFXToInches(FrontRightDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FrontRightCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{falconFXToInches(BackLeftDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BackLeftCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{falconFXToInches(BackRightDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BackRightCANCoder.GetPosition()})}
  });
  frc::SmartDashboard::PutNumber("X ", pose.X().value());
  frc::SmartDashboard::PutNumber("Y ", pose.Y().value());
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif