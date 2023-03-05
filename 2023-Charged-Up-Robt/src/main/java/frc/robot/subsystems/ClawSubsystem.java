// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ClawSubsystem extends SubsystemBase {
public WPI_TalonSRX wristMotor = new WPI_TalonSRX(Constants.Arm.wristMotor);
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void closeWrist(double power){
    wristMotor.set(ControlMode.PercentOutput, power);
  }
}
