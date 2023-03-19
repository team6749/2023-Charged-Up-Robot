// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SelfBalance extends CommandBase {

  /** Creates a new SelfBalance. */
  private SwerveDriveSubsystem subsystem;
  double accelerationX;

  double thresholdLevel = 1;
  double offsetLevel = 2.4;
  PIDController pidController = new PIDController(0.0215, 0.005, 0.004);
  Timer timer = new Timer();

  public SelfBalance(SwerveDriveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    pidController.setIntegratorRange(-0.1, 0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // uses roborio gyro angle to level robot
    accelerationX = ((SwerveDriveSubsystem.gyro.getXComplementaryAngle() + offsetLevel) + (accelerationX)) / 2;
    SmartDashboard.putNumber("accY", accelerationX);
    // sets the chassis speeds to pid calculated accelX in order to have minimal
    // movement based on the angle of the platform
    subsystem.setDesiredChassisSpeeds(new ChassisSpeeds(pidController.calculate(accelerationX), 0, 0));

    // if the angle is bigger than the threshold angle, reset the timer
    if (Math.abs(accelerationX) > (thresholdLevel)) {
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pidController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the angle is within the threshold and the timer has elapsed for 2.15 sec,
    // stop trying to balance
    return (Math.abs(accelerationX) < (thresholdLevel)) && timer.hasElapsed(2.15);
  }
}
