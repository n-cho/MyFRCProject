// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Created by Nathan Cho on 2023-02-04.
//

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <Robot.h>
#include <DeviceManager.h>

// Locations for the swerve drive modules relative to the robot center.
// Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
const frc::Translation2d kFrontLeftLocation{11.625_in, 7.25_in};
const frc::Translation2d kFrontRightLocation{11.625_in, -7.25_in};
const frc::Translation2d kBackLeftLocation{-11.625_in, 7.25_in};
const frc::Translation2d kBackRightLocation{-11.625_in, -7.25_in};

// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> kinematics{
  kFrontLeftLocation, kFrontRightLocation,
  kBackLeftLocation, kBackRightLocation
};

// Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.
// frc::SwerveDriveOdometry<4> m_odometry{
//   kinematics,
//   Gyro.GetRotation2d(),
//   {FrontLeftDrive.GetPosition(), FrontRightDrive.GetPosition(),
//   BackLeftDrive.GetPosition(), BackRightDrive.GetPosition()},
//   frc::Pose2d{0_m, 0_m, 0_rad}
//   };

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

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
int main() {
  return frc::StartRobot<Robot>();
}
#endif