// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveDirectionCommand extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final ChassisSpeeds chassisSpeeds;

  /** Creates a new DriveDirectionCommand. */
  public DriveDirectionCommand(ChassisSpeeds chassisSpeeds, DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.chassisSpeeds = chassisSpeeds;
    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Boolean backwardRelative = isRobotRelativeBackwardSupplier.getAsBoolean();

    drivebaseSubsystem.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
