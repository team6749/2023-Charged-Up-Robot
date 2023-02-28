// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmMiddle extends CommandBase {
  ArmSubsystem subsystem;
  double middleDegrees;
  /** Creates a new MoveMiddleArn. */
  public MoveArmMiddle(ArmSubsystem _subsystem, double middle) {
    subsystem = _subsystem;
    addRequirements(subsystem);
    middleDegrees = middle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.moveMiddleWithoutEncoder(middleDegrees);
    // subsystem.moveMiddleArm(middleDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
