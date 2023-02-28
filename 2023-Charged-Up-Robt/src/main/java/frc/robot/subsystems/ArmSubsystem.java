// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  //motors and encoders for the arm, each segment of the arm (base, middle, and claw) have 
  //both an arm and encoder
  double baseAngle;
  double middleAngle;
  double clawAngle;

  double baseMotorPower;
  double middleMotorPower;
  double clawMotorPower;

  WPI_TalonFX baseMotor = new WPI_TalonFX(Constants.Arm.armBaseMotor);
  // DutyCycleEncoder baseEncoder = new DutyCycleEncoder(Constants.Arm.armBaseEncoder);
  

  WPI_TalonFX middleMotor = new WPI_TalonFX(Constants.Arm.armMiddleMotor);
  // DutyCycleEncoder middleEncoder = new DutyCycleEncoder (Constants.Arm.armMiddleEncoder);

  WPI_TalonFX clawMotor = new WPI_TalonFX(Constants.Arm.armClawMotor);
  // DutyCycleEncoder clawEncoder = new DutyCycleEncoder(Constants.Arm.armClawEncoder);

    //double variables to measure the angle position of the arm in degrees
 

    
    ProfiledPIDController baseMotorController = new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(5/4,3));
    ProfiledPIDController middleMotorController = new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(5/4,3));
    ProfiledPIDController clawMotorController = new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(5/4,3));

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    baseMotor.configFactoryDefault();
    
    baseMotor.setNeutralMode(NeutralMode.Brake);
    middleMotor.setNeutralMode(NeutralMode.Brake);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    baseMotor.set(ControlMode.PercentOutput, 0);

  }

  // public double getBaseAnglePosition() {
  //   return baseAngle = (baseEncoder.getAbsolutePosition() * 360);
  // }
  // public double getMiddleAnglePosition() {
  //   return middleAngle = (middleEncoder.getAbsolutePosition() * 360);
  // }
  // public double getClawAnglePosition() {
  //   return clawAngle = (clawEncoder.getAbsolutePosition() *360);
  // }


  // public double getBaseDegrees(){
  //   return getBaseAnglePosition();
  // }
  // public double getMiddleDegrees(){
  //   return getMiddleAnglePosition();
  // }
  // public double getClawDegrees(){
  //   return getClawAnglePosition();
  // }

  // public void moveBaseOfArm(double degrees){
  //   baseMotor.set(ControlMode.PercentOutput, baseMotorController.calculate(getBaseAnglePosition(), degrees));
  // }

  // public void moveMiddleArm(double degrees){
  //   middleMotor.set(ControlMode.PercentOutput, middleMotorController.calculate(getMiddleAnglePosition(), degrees));
  //   }
  
  // public void moveClaw(double degrees){
  //   clawMotor.set(ControlMode.PercentOutput, clawMotorController.calculate(getClawAnglePosition(), degrees));
  // }

  public void moveBaseWithoutEncoder(double power){
    baseMotor.set(ControlMode.PercentOutput, power);
  }

  public void moveMiddleWithoutEncoder(double power){
    middleMotor.set(ControlMode.PercentOutput, power);
  }
  public void moveClawWithoutEncoder(double power){
    clawMotor.set(ControlMode.PercentOutput, power);
  }
  }

