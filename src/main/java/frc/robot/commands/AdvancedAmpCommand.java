// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AdvancedAmpCommand extends SequentialCommandGroup {
    public AdvancedAmpCommand(
      ShooterSubsystem shooterSubsystem,
      RGBSubsystem rgbSubsystem,
      DrivebaseSubsystem drivebaseSubsystem) {
    addCommands(
      new ShootCommand(shooterSubsystem, true).withTimeout(2),
      new DriveDirectionCommand(
        DrivebaseSubsystem.produceChassisSpeeds(
          true, 0, 1, 0, drivebaseSubsystem.getDriverGyroscopeRotation())
          ,drivebaseSubsystem)
          .withTimeout(0.4),
      new DriveDirectionCommand(
        DrivebaseSubsystem.produceChassisSpeeds(
          true, 0, -1, 0, drivebaseSubsystem.getDriverGyroscopeRotation())
          ,drivebaseSubsystem)
          .withTimeout(0.4));
  }
}
