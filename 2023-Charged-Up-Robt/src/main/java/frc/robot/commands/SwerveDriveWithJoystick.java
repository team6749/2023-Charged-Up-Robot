// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
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

    double verticalDirectionSpeed = -limitedJoystickInput(sdJoystick.getY());
    double horizontalDirectionSpeed = -limitedJoystickInput(sdJoystick.getX());
    double rotationalSpeed = (2.0*sdJoystick.getTwist());
    
    
    //x joystick deadzone
    if(Math.abs(horizontalDirectionSpeed) < (0.25) ) {
      horizontalDirectionSpeed = 0;
    }
    
    //y joystick deadzone
    if(Math.abs(verticalDirectionSpeed) < (0.25)) {
      verticalDirectionSpeed = 0;
    }
    
    if(Math.abs(rotationalSpeed) < (0.2) ) {
        rotationalSpeed = 0;
    }
    System.out.println("X: " + horizontalDirectionSpeed);
    System.out.println("Y: " + verticalDirectionSpeed);
    System.out.println("Rot: " + rotationalSpeed);
    switch(selectedDriveMode){
      case ("Robot Oriented"):
          //put robot oriented drive here.
          desiredSpeeds = new ChassisSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed);
          break;
      
      case ("Field Oriented"):
          //put field oriented drive here.
          desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed, swerveDriveSubsystem.getRotation());
          break;
      
    }

    swerveDriveSubsystem.setDesiredChassisSpeeds(desiredSpeeds);
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
