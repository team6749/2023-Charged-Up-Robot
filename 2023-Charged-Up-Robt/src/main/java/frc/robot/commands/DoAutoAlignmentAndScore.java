// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DoAutoAlignmentAndScore extends CommandBase {

  private Command myCommand;
  //This is really bad practice :D
  RobotContainer container;

  /** Creates a new DoAutoAlignment. */
  public DoAutoAlignmentAndScore(RobotContainer container) {
    this.container = container;

    // this is especially bad practice -spencer
    addRequirements(container._ArmSubsystem);
    addRequirements(container._ClawSubsystem);
    addRequirements(container._SwerveDrivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myCommand = Constants.ArmCommands.moveArmIdle(container._ArmSubsystem)
    .andThen(container.stationSelector.getSelected())
    .andThen(container.scoringSelector.getSelected());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return myCommand.isFinished();
  }
}
