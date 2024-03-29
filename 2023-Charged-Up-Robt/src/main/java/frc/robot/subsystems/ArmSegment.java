// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmSegment extends PIDSubsystem {

  public WPI_TalonFX motor;
  DutyCycleEncoder encoder;

  double minRange;
  double maxRange;
  double maxOutput;

  /** Creates a new ArmSegment. */
  public ArmSegment(double maxOutput, double minRange, double maxRange, boolean invert, double offset, PIDController controller, int motorID, int encoderID) {
    super(controller);

    this.maxOutput = maxOutput;

    motor = new WPI_TalonFX(motorID);
    encoder = new DutyCycleEncoder(encoderID);
  
    this.minRange = minRange;
    this.maxRange = maxRange;

    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(invert); // TODO see how this works

    encoder.setDistancePerRotation(360);
    if (encoder.getPositionOffset() == 0) {
      encoder.setPositionOffset(offset / 360);
    }
    
  
    //degrees
    controller.setTolerance(2);

    //By default do not move
    disable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if (output > maxOutput) {
      output = maxOutput;
    }
    if (output < -maxOutput) {
      output = -maxOutput;
    }

    if(getMeasurement() < minRange - 30) {
      DriverStation.reportError(getName() + " encoder is way out of range (+) not moving...", true);
      motor.set(ControlMode.PercentOutput, 0);
      return;
    }
    if(getMeasurement() > maxRange + 30) {
      DriverStation.reportError(getName() + " encoder is way out of range (-) not moving...", true);
      motor.set(ControlMode.PercentOutput, 0);
      return;
    }

    SmartDashboard.putNumber(getName() + " PID", output);
    motor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + " Encoder ABS", encoder.getAbsolutePosition());
    SmartDashboard.putNumber(getName() + " Encoder DIST", encoder.get());
    
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
    double value = encoder.getDistance();
    //TODO this is REALLY JANK like REALLY jank to try and solve a hardware setup thing
    //i have no idea if it even works
    if(value < -150) {
      DriverStation.reportWarning(getName() + " is using ov erride +360 kinda sus", false);
      value += 360;
    }
    return value;
  }
}
