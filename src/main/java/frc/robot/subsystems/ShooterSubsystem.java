// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX upper;
  private final TalonFX lower;
  private double upperSpeed;
  private double lowerSpeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    upper = new TalonFX(60);
    lower = new TalonFX(20);
    lower.clearStickyFaults();
    upper.clearStickyFaults();
  }

  // Set upper speed

  public void setSpeed(double upperSpeed, double lowerSpeed) {
    this.upperSpeed = upperSpeed;
    this.lowerSpeed = lowerSpeed;
    // supposed to be slower but idk by how much :skull:
  }
  public void setSpeed(double speed) {
    this.upperSpeed = speed;
    this.lowerSpeed = speed;
    // supposed to be slower but idk by how much :skull:
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upper.setVoltage(upperSpeed);
    lower.setVoltage(lowerSpeed);

  }
}
