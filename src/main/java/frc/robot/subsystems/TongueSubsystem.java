package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive.Dims;
import frc.robot.commands.TongueCommand;
import frc.robot.subsystems.DrivebaseSubsystem.Modes;
import frc.util.Util;

public class TongueSubsystem extends SubsystemBase {
    
    private final 
    private final TalonSRX tonguemotor;   
    public enum Modes {
        INTAKE,
        OUTTAKE
    }
    public enum TongueMode {
        public final TonguePowers tonguePowers;
        private TongueMode(TonguePowers tonguePowers) {
            this.tonguePowers = tonguePowers;
        }
    }

    public record TonguePowers(double roller, double accelerator) {
        public TonguePowers(double roller){
            this.roller = roller;
        }
    }
    public TongueSubsystem() {
        tonguemotor = new TalonSRX(3);
  
    }
}
 



