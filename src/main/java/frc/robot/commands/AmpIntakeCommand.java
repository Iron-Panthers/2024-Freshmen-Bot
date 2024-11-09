// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.AmpSubsystem.AmpMode;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;

public class AmpIntakeCommand extends Command {
    private AmpSubsystem ampSubsystem;
    private RGBSubsystem rgbSubsystem;
    private Optional <RGBMessage> message;
  /** Creates a new DefenseModeCommand. */
  public AmpIntakeCommand(AmpSubsystem ampSubsystem, RGBSubsystem rgbSubsystem) {
    this.ampSubsystem = ampSubsystem;
    this.rgbSubsystem = rgbSubsystem;
    message = Optional.empty();
    addRequirements(ampSubsystem, rgbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        ampSubsystem.setAmpMode(AmpMode.INTAKE);
        message = Optional.of (rgbSubsystem.showMessage(
                  Constants.Lights.Colors.ORANGE,
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
    ampSubsystem.setAmpMode(AmpMode.IDLE);
    message.ifPresent(RGBMessage::expire);
    message = Optional.empty();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
