// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  double baseMotorPower;
  double clawMotorPower;

  public ArmSegment baseSegment = new ArmSegment(-45, 100, true, Constants.Arm.baseOffset, new PIDController(0.05, 0, 0));

  WPI_TalonFX clawMotor = new WPI_TalonFX(Constants.Arm.clawMotor);
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    baseSegment.setName("Base Segment");
    baseSegment.disable();
    // baseSegment.setSetpoint(90);
    // baseSegment.getController().atSetpoint();
  }


  @Override
  public void periodic() {
    moveClawWithoutEncoder(0.1);
  }

  // public double getBaseAnglePosition() {
  //   return baseTargetAngle = ( * 360) + Constants.Arm.baseOffset;
  // }
  // public double getMiddleAnglePosition() {
  //   return middleAngle = (middleEncoder.getAbsolutePosition() * 360);
  // }
  // public double getClawAnglePosition() {
  //   return clawAngle = (clawEncoder.getAbsolutePosition() *360);
  // }


  public void moveBaseWithoutEncoder(double power){
    System.out.println(power);
    baseSegment.motor.set(ControlMode.PercentOutput, power);
  }

  public void moveClawWithoutEncoder(double power){
    clawMotor.set(ControlMode.PercentOutput, power);
  }
  }

