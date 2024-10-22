package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
import frc.robot.Constants.Amp;
import frc.robot.commands.AmpCommand;
import frc.robot.subsystems.DrivebaseSubsystem.Modes;
import frc.util.Util;

public class AmpSubsystem extends SubsystemBase {
    private final TalonSRX ampMotor;  
    private AmpMode ampMode; 
    public enum AmpMode {
        INTAKE(Amp.Modes.INTAKE),
        IDLE(Amp.Modes.IDLE),
        OUTTAKE(Amp.Modes.OUTTAKE);
        public final AmpPowers ampPowers;
        private AmpMode(AmpPowers ampPowers) {
            this.ampPowers = ampPowers;
        }
    }

    public record AmpPowers(double roller) {
        public AmpPowers(double roller){
            this.roller = roller;
        }
    }

    public void AmpSubsystem() { 
        ampMotor = new TalonSRX(Amp.Ports.CANCODER_PORT);
        ampMode = AmpMode.IDLE();
  
    }
    public void setAmpMode(AmpMode ampMode) {
        this.ampMode = ampMode;
      }
    @Override
    public void periodic() {
    ampMotor.set(ampMode.ampPowers.roller());
    }
}


