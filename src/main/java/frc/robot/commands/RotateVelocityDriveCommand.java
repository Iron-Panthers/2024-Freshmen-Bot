// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;

/**
 * This command takes a drive stick, and then a velocity double to drive and rotate the robot. The
 * velocity is meters per second.
 */
public class RotateVelocityDriveCommand extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final BooleanSupplier isRobotRelativeForwardSupplier;
  // private final BooleanSupplier isRobotRelativeBackwardSupplier;

  /** Creates a new RotateVelocityDriveCommand. */
  public RotateVelocityDriveCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier isRobotRelativeForwardSupplier) {
    // BooleanSupplier isRobotRelativeBackwardSupplier) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.isRobotRelativeForwardSupplier = isRobotRelativeForwardSupplier;
    // this.isRobotRelativeBackwardSupplier = isRobotRelativeBackwardSupplier;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = translationXSupplier.getAsDouble();
    double y = translationYSupplier.getAsDouble();
    double rot = rotationSupplier.getAsDouble();
    double rotSign = Math.copySign(1, rot);

    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented
    // movement

    drivebaseSubsystem.driveAngle(
      new Pair<Double, Double>(x,y),drivebaseSubsystem.getTargetAngle()+5*rotSign*Math.pow(rot*rotSign, 1.5));
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
