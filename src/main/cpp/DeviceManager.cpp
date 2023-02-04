//
// Created by Nathan Cho on 2023-02-04.
//

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