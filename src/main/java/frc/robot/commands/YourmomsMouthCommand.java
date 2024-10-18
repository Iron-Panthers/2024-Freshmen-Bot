// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.YourmomsMouthSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class YourmomsMouthCommand extends InstantCommand {

    private final YourmomsMouthSubsystem yourmomsMouth;

    public YourmomsMouthCommand(
            YourmomsMouthSubsystem yourmomsMouth) {
        this.yourmomsMouth = yourmomsMouth;
        addRequirements(yourmomsMouth);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }
}
