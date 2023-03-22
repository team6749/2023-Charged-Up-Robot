// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  double baseMotorPower;
  double clawMotorPower;

  public ArmSegment baseSegment = new ArmSegment(0.25, -45, 95, true, Constants.Arm.baseOffset, new PIDController(0.05, 0, 0), Constants.Arm.baseMotor, Constants.Arm.baseEncoder);
  public ArmSegment clawSegment = new ArmSegment(0.25, -95, 135, true, Constants.Arm.clawOffset, new PIDController(0.05, 0, 0), Constants.Arm.clawMotor, Constants.Arm.clawEncoder);

  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    baseSegment.setName("Base Segment");
    clawSegment.setName("Claw Segment");
    

    // baseSegment.setSetpoint(90);
    // baseSegment.getController().atSetpoint();
  }


  @Override
  public void periodic() {
  //   moveClawSegment(0);

  }


  public void moveBaseSegment(double power){
    baseSegment.motor.set(ControlMode.PercentOutput, power);
  }

  // public void moveClawSegment(double power){
  //   clawSegment.motor.set(ControlMode.PercentOutput, power);
  // }
  }

