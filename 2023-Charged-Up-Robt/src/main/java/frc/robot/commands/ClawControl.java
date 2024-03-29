// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawControl extends CommandBase {
  ClawSubsystem m_ClawSubsystem;
  boolean m_direction;
  Timer timer;
  /** Creates a new ClawControl. */
  public ClawControl(ClawSubsystem subsystem, boolean direction) {
    m_ClawSubsystem = subsystem;
    timer = new Timer();
    addRequirements(subsystem);
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if(m_direction == true){
      m_ClawSubsystem.openSolenoid();
    } 
    if(m_direction == false){
      m_ClawSubsystem.closeSolenoid();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.1);
  }
}
