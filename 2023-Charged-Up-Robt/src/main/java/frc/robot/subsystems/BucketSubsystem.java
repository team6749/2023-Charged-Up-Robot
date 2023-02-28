// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BucketSubsystem extends SubsystemBase {
  /** Creates a new BucketSubsystem. */
  public TalonFX bucketMotor;
  public Joystick joystick;
  public BucketSubsystem(int id, Joystick joystick) {
    bucketMotor = new TalonFX(id);
    this.joystick = joystick;
    bucketMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(joystick.getRawButton(3)){
      dump();
    } else if(joystick.getRawButton(5)){
      undump();
    } else {
      none();
    }
  }
  
  public void dump(){
    bucketMotor.set(ControlMode.PercentOutput, -0.075);
  }
  public void undump(){
    bucketMotor.set(ControlMode.PercentOutput, 0.075);
  }
  public void none(){
    bucketMotor.set(ControlMode.PercentOutput, 0);
  }
}
