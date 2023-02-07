//
// Created by Nathan Cho on 2023-02-04.
//

#include "ctre/Phoenix.h"
#include "frc/XboxController.h"
#include "AHRS.h"

extern TalonFX FrontLeftDrive;
extern TalonFX FrontLeftSteering;
extern TalonFX FrontRightDrive;
extern TalonFX FrontRightSteering;
extern TalonFX BackRightDrive;
extern TalonFX BackRightSteering;
extern TalonFX BackLeftDrive;
extern TalonFX BackLeftSteering;

extern CANCoder BackLeftCANCoder;
extern CANCoder BackRightCANCoder;
extern CANCoder FrontLeftCANCoder;
extern CANCoder FrontRightCANCoder;

extern AHRS navX;

extern frc::XboxController Controller;