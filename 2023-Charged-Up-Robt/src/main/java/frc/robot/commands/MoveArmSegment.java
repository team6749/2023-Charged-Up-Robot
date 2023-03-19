// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSegment;

public class MoveArmSegment extends CommandBase {

  ArmSegment m_Segment;
  double m_setPoint;

  /** Creates a new MoveArmSegment. */
  public MoveArmSegment(ArmSegment segment, double setPoint) {
    m_Segment = segment;
    m_setPoint = setPoint;
    addRequirements(segment);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("hello");
    m_Segment.enable();
    m_Segment.setSetpoint(m_setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Segment.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Segment.getController().atSetpoint();
  }
}
