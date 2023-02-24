// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  //motors and encoders for the arm, each segment of the arm (base, middle, and claw) have 
  //both an arm and encoder
  WPI_TalonFX baseMotor = new WPI_TalonFX(Constants.Arm.armBaseMotor);
  DutyCycleEncoder baseEncoder = new DutyCycleEncoder(Constants.Arm.armBaseEncoder);

  WPI_TalonFX middleMotor = new WPI_TalonFX(Constants.Arm.armMiddleMotor);
  DutyCycleEncoder middleEncoder = new DutyCycleEncoder (Constants.Arm.armMiddleEncoder);

  WPI_TalonFX clawMotor = new WPI_TalonFX(Constants.Arm.armClawMotor);
  DutyCycleEncoder clawEncoder = new DutyCycleEncoder(Constants.Arm.armClawEncoder);

  //double variables to measure the angle position of the arm in degrees
  double baseAngle = baseEncoder.getAbsolutePosition();
  double middleAngle = middleEncoder.getAbsolutePosition();
  double clawAngle = clawEncoder.getAbsolutePosition();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    
  }

  public void moveArm(double degrees){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
