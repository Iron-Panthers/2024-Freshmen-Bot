// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class YourmomsMouthSubsystem extends SubsystemBase {

  private final TalonSRX upper;
  private final TalonSRX lower;
  private double upperSpeed;
  private double lowerSpeed;

  /** Creates a new YourmomsMouth. */
  public YourmomsMouthSubsystem() {
    upper = new TalonSRX(1);
    lower = new TalonSRX(2);

  }

  public void suckingIn() {
    upperSpeed = 1;
  }
  // Set upper speed
 
  public void spitSpeed(double speed) {
    upperSpeed = speed;
    lowerSpeed = speed*0.5;
    //supposed to be slower but idk by how much :skull:
  }
  public void gulpSpeed(double speed){
    upperSpeed = speed;
    lowerSpeed = speed*0.5;
    //supposed ot be slower but idk how much :skull:
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upper.set(TalonSRXControlMode.PercentOutput, upperSpeed);
    lower.set(TalonSRXControlMode.PercentOutput, lowerSpeed);

  }
}
