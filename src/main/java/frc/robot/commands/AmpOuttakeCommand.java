// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.AmpSubsystem.AmpMode;

public class AmpOuttakeCommand extends Command {
    AmpSubsystem ampSubsystem;
  /** Creates a new DefenseModeCommand. */
  public AmpOuttakeCommand(AmpSubsystem ampSubsystem) {
    this.ampSubsystem = ampSubsystem;
    addRequirements(ampSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ampSubsystem.setAmpMode(AmpMode.OUTTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampSubsystem.setAmpMode(AmpMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
