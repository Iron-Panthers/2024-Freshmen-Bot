package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

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
import frc.robot.commands.AmpOuttakeCommand;
import frc.robot.subsystems.DrivebaseSubsystem.Modes;
import frc.util.Util;

public class AmpSubsystem extends SubsystemBase {
    private final TalonSRX ampMotor;  
    private AmpMode ampMode; 
    public enum AmpMode {
        INTAKE,
        IDLE,
        OUTTAKE;
    }

    public AmpSubsystem() { 
        ampMotor = new TalonSRX(Amp.Ports.AMP_MOTOR_PORT);
        ampMode = AmpMode.IDLE;
    }
    public void setAmpMode(AmpMode ampMode) {
        this.ampMode = ampMode;
    }
    @Override
    public void periodic() {
        double power = 0;
        switch (ampMode) {
            case IDLE:
                power = 0;
                break;
            case INTAKE:
                power = 0.5;
                break;
            case OUTTAKE:
                power = -0.5;
                break;
        }

    ampMotor.set(TalonSRXControlMode.PercentOutput, power);
    }
}