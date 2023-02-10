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

TalonFX FLDrive{kFLDriveID};
TalonFX FLSteering{kFLSteeringID};
TalonFX FRDrive{kFRDriveID};
TalonFX FRSteering{kFRSteeringID};
TalonFX BLDrive{kBLDriveID};
TalonFX BLSteering{kBLSteeringID};
TalonFX BRDrive{kBRDriveID};
TalonFX BRSteering{kBRSteeringID};

CANCoder BLCANCoder{kBLCANCoderID};
CANCoder BRCANCoder{kBRCANCoderID};
CANCoder FLCANCoder{kFLCANCoderID};
CANCoder FRCANCoder{kFRCANCoderID};

AHRS navX{frc::SPI::kMXP};

frc::XboxController controller{kControllerID};

// Locations for the swerve drive modules relative to the robot center.
// Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
const frc::Translation2d kFLLocationFromCenter{11.625_in, 7.25_in};
const frc::Translation2d kFRLocationFromCenter{11.625_in, -7.25_in};
const frc::Translation2d kBLLocationFromCenter{-11.625_in, 7.25_in};
const frc::Translation2d kBRLocationFromCenter{-11.625_in, -7.25_in};

// Creating kinematics object using the module locations.
frc::SwerveDriveKinematics<4> kinematics{
  kFLLocationFromCenter, 
  kFRLocationFromCenter,
  kBLLocationFromCenter, 
  kBRLocationFromCenter
};

// Convert SelectedSensorPosition (degrees) to Inches.
units::length::inch_t TalonFXToInches(double selectedSensorPosition) {
  return units::length::inch_t{(selectedSensorPosition * 4 * M_PI) / 180};
}

// Creating odometry object from the kinematics object, navX rotation, swerve module positions.
frc::SwerveDriveOdometry<4> odometry{
  kinematics,
  navX.GetRotation2d(),
  {
    frc::SwerveModulePosition{TalonFXToInches(FLDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(FRDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(BLDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(BRDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
  },
  frc::Pose2d{0_m, 0_m, 0_deg}
};

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  // Get the rotation of the robot from the gyro.
  frc::Rotation2d rotation = navX.GetRotation2d();

  // Update the pose.
  auto pose = odometry.Update(rotation,
  {
    frc::SwerveModulePosition{TalonFXToInches(FLDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FLCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(FRDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{FRCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(BLDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BLCANCoder.GetPosition()})}, 
    frc::SwerveModulePosition{TalonFXToInches(BRDrive.GetSelectedSensorPosition()), frc::Rotation2d(units::angle::degree_t{BRCANCoder.GetPosition()})}
  });

  // Display pose and rotation on SmartDashboard.
  frc::SmartDashboard::PutNumber("X ", pose.X().value());
  frc::SmartDashboard::PutNumber("Y ", pose.Y().value());
  frc::SmartDashboard::PutNumber("theta ", rotation.Degrees().value());
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