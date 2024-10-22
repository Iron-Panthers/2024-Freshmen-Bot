// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

  private ShooterSubsystem shooterSubsystem;
  private boolean amp;

  /** Creates a new SuckIn. */
  public ShootCommand(ShooterSubsystem shooterSubsystem, boolean amp) {
    this.shooterSubsystem = shooterSubsystem;
    this.amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (amp){
      shooterSubsystem.setSpeed(Constants.Mouthy.SHOOT_AMP_SPEED_UPPER, Constants.Mouthy.SHOOT_AMP_SPEED_LOWER);
    }
    else{
      shooterSubsystem.setSpeed(Constants.Mouthy.SHOOT_SPEAKER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
