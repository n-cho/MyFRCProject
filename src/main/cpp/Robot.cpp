// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Created by Nathan Cho on 2023-01-14.
//

#include "Robot.h"

#include "ctre/Phoenix.h"
#include "frc/XboxController.h"

TalonFX FrontLeftDrive = 3;
TalonFX FrontLeftSteering = 7;
TalonFX FrontRightDrive = 1;
TalonFX FrontRightSteering = 5;
TalonFX BackRightDrive = 4;
TalonFX BackRightSteering = 8;
TalonFX BackLeftDrive = 2;
TalonFX BackLeftSteering = 6;

CANCoder BackLeftCANCoder = 10;
CANCoder BackRightCANCoder = 12;
CANCoder FrontLeftCANCoder = 11;
CANCoder FrontRightCANCoder = 9;

frc::XboxController Controller = frc::XboxController(0);

const auto kControlMode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;

int Math() {
    return 1;
}

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

Robot::Robot() {
    AddPeriodic([&] { Math(); }, 10_ms, 0_ms);
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif