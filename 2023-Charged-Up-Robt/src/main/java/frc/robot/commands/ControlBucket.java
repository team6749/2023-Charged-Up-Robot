// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BucketSubsystem;

public class ControlBucket extends CommandBase {
  /** Creates a new ControlBucket. */
  BucketSubsystem subsystem;
  double powerToSet;
  public ControlBucket(BucketSubsystem subsystem, double powerToSet) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.powerToSet = powerToSet;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  subsystem.bucketMotor.set(ControlMode.PercentOutput, powerToSet);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
