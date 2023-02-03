// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SelfBalance extends CommandBase {
  /** Creates a new SelfBalance. */
  private SwerveDriveSubsystem subsystem;
  double accY;
  double thresholdLevel = 4;
  double offsetLevel = 3.7;
  PIDController pidController = new PIDController(0.0175,0,0);
  Timer timer = new Timer();

  public SelfBalance(SwerveDriveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //uses roborio accel y to level robot
    accY = ((SwerveDriveSubsystem.gyro.getYComplementaryAngle() + offsetLevel) + (accY))/2;
    SmartDashboard.putNumber("accY", accY);
    subsystem.setDesiredChassisSpeeds(new ChassisSpeeds(-pidController.calculate(accY), 0, 0));
    if(Math.abs(accY) > (thresholdLevel)){
      timer.reset();
    } else{
      subsystem.setDesiredChassisSpeeds(new ChassisSpeeds(0,0,0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (Math.abs(accY) < (thresholdLevel)) && timer.hasElapsed(1.5);
  }
}