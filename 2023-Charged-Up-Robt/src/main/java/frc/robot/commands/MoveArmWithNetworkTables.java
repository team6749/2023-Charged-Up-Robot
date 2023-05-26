// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmWithNetworkTables extends CommandBase {
  ArmSubsystem _subsystem;
  /** Creates a new MoveArmWithNetworkTables. */
  public MoveArmWithNetworkTables(ArmSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    _subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _subsystem.baseSegment.enable();
    _subsystem.clawSegment.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    NetworkTableInstance table = NetworkTableInstance.getDefault();

    NetworkTable table2 = table.getTable("SmartDashboard");
    double[] array = table2.getEntry("vision_measurement").getDoubleArray(new double[]{0d, 0d});

    double wrist = array[1];
    double elbow = array[0];

  
    wrist = Math.round(wrist);
    elbow = Math.round(elbow);

    // _subsystem.clawSegment.setSetpoint(wrist);
    _subsystem.baseSegment.setSetpoint(elbow);    

    System.out.println(elbow);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _subsystem.baseSegment.disable();
    _subsystem.clawSegment.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
