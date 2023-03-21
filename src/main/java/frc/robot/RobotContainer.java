// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;

public class RobotContainer {

    public final SwerveDriveSubsystem drive;

    public RobotContainer() {
        drive = new SwerveDriveSubsystem();
        configureBindings();
    }

    private void configureBindings() {

    }

    public Command autonomousInit() {
        return Commands.print("No autonomous command configured");
    }

    public Command teleopInit() {
        return Commands.print("No teleop init command configured");
    }
}
