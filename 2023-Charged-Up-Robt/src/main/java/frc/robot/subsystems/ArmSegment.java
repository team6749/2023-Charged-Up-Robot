// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ArmSegment extends PIDSubsystem {

  WPI_TalonFX motor = new WPI_TalonFX(Constants.Arm.baseMotor);
  DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Arm.baseEncoder);

  double minRange;
  double maxRange;

  /** Creates a new ArmSegment. */
  public ArmSegment(double minRange, double maxRange, boolean invert, double offset, PIDController controller) {
    super(controller);
  
    this.minRange = minRange;
    this.maxRange = maxRange;

    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(invert); // TODO see how this works

    encoder.setDistancePerRotation(360);
    encoder.setPositionOffset(offset / 360);
  
    //degrees
    controller.setTolerance(2);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if (output > 0.1) {
      output = 0.1;
    }
    if (output < -0.1) {
      output = -0.1;
    }
    SmartDashboard.putNumber(getName() + " PID", output);
    motor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + " Encoder", getMeasurement());

    // Do Range limits before the built in super method calls useOutput
    if (getSetpoint() < minRange) {
      setSetpoint(minRange);
    }
    if (getSetpoint() > maxRange) {
      setSetpoint(maxRange);
    }

    super.periodic();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getDistance();
  }
}
