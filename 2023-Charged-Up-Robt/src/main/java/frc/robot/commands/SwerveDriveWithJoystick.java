// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveWithJoystick extends CommandBase {
  /** Creates a new SwerveDriveWithJoystick. */
  public SendableChooser<String> orientation = new SendableChooser<String>(); 
  private String selectedDriveMode;
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private Joystick sdJoystick;
  public ChassisSpeeds desiredSpeeds;

  public SwerveDriveWithJoystick(SwerveDriveSubsystem subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDriveSubsystem = subsystem;
    sdJoystick = joystick;
    orientation.setDefaultOption("Robot Oriented", "Robot Oriented");
    orientation.addOption("Field Oriented", "Field Oriented");
    SmartDashboard.putData(orientation);
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    selectedDriveMode = orientation.getSelected();

    double joystickRotation = sdJoystick.getTwist();
    if(Math.abs(joystickRotation) < (0.30) ) {
      joystickRotation = 0;
  }

    double verticalDirectionSpeed = limitedJoystickInput(-sdJoystick.getY()) * 2;
    double horizontalDirectionSpeed = limitedJoystickInput(-sdJoystick.getX()) * 2;
    double rotationalSpeed = limitedJoystickInput(-joystickRotation) * 3;
    
    if(sdJoystick.getMagnitude() < 0.15) {
      horizontalDirectionSpeed = 0;
      verticalDirectionSpeed = 0;
    }
    
    
    // System.out.println("X: " + horizontalDirectionSpeed);
    // System.out.println("Y: " + verticalDirectionSpeed);
    // System.out.println("Rot: " + rotationalSpeed);

    
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
          if(DriverStation.getAlliance() != Alliance.Red){
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed, swerveDriveSubsystem.getPose2d().getRotation());
          } else {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-verticalDirectionSpeed, -horizontalDirectionSpeed, rotationalSpeed, swerveDriveSubsystem.getPose2d().getRotation());
          }
          break;
    }
    if(verticalDirectionSpeed == 0 && horizontalDirectionSpeed == 0 && rotationalSpeed == 0){
      swerveDriveSubsystem.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
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

}