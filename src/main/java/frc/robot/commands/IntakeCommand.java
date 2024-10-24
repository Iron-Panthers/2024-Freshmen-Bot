// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {

  private ShooterSubsystem shooterSubsystem;
  private boolean amp;
  private RGBSubsystem rgbSubsystem;
  private Optional <RGBMessage> message;


  /** Creates a new SuckIn. */
  public IntakeCommand(ShooterSubsystem shooterSubsystem, boolean amp, RGBSubsystem rgbSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.amp = amp;
    this.rgbSubsystem = rgbSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    message = Optional.empty();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSpeed(Constants.Mouthy.INTAKE_SPEED);
    message = Optional.of (rgbSubsystem.showMessage(
            Constants.Lights.Colors.WHITE,
            RGBSubsystem.PatternTypes.STROBE,
            RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(amp? Constants.Mouthy.SHOOT_AMP_SPEED_UPPER :Constants.Mouthy.SHOOT_SPEAKER_SPEED, 0);
    message.ifPresent(RGBMessage::expire);
    message = Optional.empty();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
