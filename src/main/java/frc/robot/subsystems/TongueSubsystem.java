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
import frc.robot.Constants.Tongue;
import frc.robot.commands.TongueCommand;
import frc.robot.subsystems.DrivebaseSubsystem.Modes;
import frc.util.Util;

public class TongueSubsystem extends SubsystemBase {
    private final TalonSRX tongueMotor;  
    private TongueMode tongueMode; 
    public enum TongueMode {
        INTAKE(Tongue.Modes.INTAKE),
        IDLE(Tongue.Modes.IDLE),
        OUTTAKE(Tongue.Modes.OUTTAKE);
        public final TonguePowers tonguePowers;
        private TongueMode(TonguePowers tonguePowers) {
            this.tonguePowers = tonguePowers;
        }

    public record TonguePowers(double roller) {
        public TonguePowers(double roller){
            this.roller = roller;
        }
    }

    public TongueSubsystem() {
        tongueMotor = new TalonSRX(Tongue.Ports.CANCODER_PORT);
        tongueMode = TongueMode.IDLE();
  
    }
    public void setTongueMode(TongueMode tongueMode) {
        this.tongueMode = tongueMode;
      }
    @Override
    public void periodic() {
    tongueMotor.set(tongueMode.tonguePowers.roller());
    }
 }
}


