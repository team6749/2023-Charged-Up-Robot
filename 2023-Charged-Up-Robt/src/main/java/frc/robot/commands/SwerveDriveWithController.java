// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveWithController extends CommandBase {
  /** Creates a new SwerveDriveWithJoystick. */
  
  private String selectedDriveMode;
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private XboxController controller;
  public ChassisSpeeds desiredSpeeds;

  SlewRateLimiter horizontalLimiter = new SlewRateLimiter(3);
  SlewRateLimiter verticalLimiter = new SlewRateLimiter(3);
  SlewRateLimiter rotationLimiter = new SlewRateLimiter(15);

  public SwerveDriveWithController(SwerveDriveSubsystem subsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDriveSubsystem = subsystem;
    this.controller = controller;
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    selectedDriveMode = swerveDriveSubsystem.orientation.getSelected();

    double joystickRotation = controller.getRightX();
    if(Math.abs(joystickRotation) < 0.10 ) {
      joystickRotation = 0;
  }
  System.out.println(controller.getPOV());

    double verticalDirectionSpeed = limitedJoystickInput(-controller.getLeftY());
    double horizontalDirectionSpeed = limitedJoystickInput(-controller.getLeftX());
    double rotationalSpeed = limitedJoystickInput(-joystickRotation);

    


    if(magnitude(verticalDirectionSpeed, horizontalDirectionSpeed) < 0.15) {
      horizontalDirectionSpeed = 0;
      verticalDirectionSpeed = 0;
    }

    verticalDirectionSpeed = verticalLimiter.calculate(verticalDirectionSpeed * 2);
    horizontalDirectionSpeed = horizontalLimiter.calculate(horizontalDirectionSpeed *2);
    rotationalSpeed = rotationLimiter.calculate(rotationalSpeed * 4.5);
    
    SmartDashboard.putNumber("vertical driving speed", verticalDirectionSpeed);
    SmartDashboard.putNumber("horizontal driving speed", horizontalDirectionSpeed);
    SmartDashboard.putNumber("Joystick twist ", rotationalSpeed);
    
    switch(selectedDriveMode){
      case ("Robot Oriented"):
          //put robot oriented drive here.
            desiredSpeeds = new ChassisSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed);
            break;
      case ("Field Oriented"):
          //put field oriented drive here.
          desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed, swerveDriveSubsystem.getPose2d().getRotation());
          break;
    }
    if(verticalDirectionSpeed == 0 && horizontalDirectionSpeed == 0 && rotationalSpeed == 0){
      swerveDriveSubsystem.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      });
      } else {
        swerveDriveSubsystem.setDesiredChassisSpeeds(desiredSpeeds);
      }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //limiting joystick sense
  public double limitedJoystickInput(double input){
    //C is the constant multiplier in the limited output eq
    double C = 0.8;
    double limitedOutput = (C * (Math.pow(input, 3))) + ((1 - C) * input);
    return limitedOutput;
  }
    
  double magnitude(double x, double y) {
    final double xSquared   = Math.pow(x, 2);
    final double ySquared   = Math.pow(y, 2);
    
    return Math.sqrt(xSquared + ySquared);
  }
}