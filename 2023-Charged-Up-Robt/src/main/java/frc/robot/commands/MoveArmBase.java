// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmBase extends CommandBase {
  ArmSubsystem subsystem;
  double base;
  /** Creates a new MoveArm. */
  public MoveArmBase(ArmSubsystem _subsystem, double baseDegrees) {
    subsystem = _subsystem;
    addRequirements(subsystem);
    base = baseDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.moveBaseWithoutEncoder(base);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.moveBaseWithoutEncoder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(subsystem.getBaseDegrees() == base){
    return false;
    // } return false;
  }
}
