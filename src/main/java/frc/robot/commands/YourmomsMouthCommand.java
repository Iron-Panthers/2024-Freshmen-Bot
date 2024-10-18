// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.YourmomsMouth;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class YourmomsMouthCommand extends InstantCommand {
    private final YourmomsMouth yourmomsMouthSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    private void addRequirements(YourmomsMouth yourmomsMouthSubsystem) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addRequirements'");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
}
