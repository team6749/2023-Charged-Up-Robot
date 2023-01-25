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
    orientation.setDefaultOption("Field Oriented", "Field Oriented");
    orientation.addOption("Robot Oriented", "Robot Oriented");
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

    double verticalDirectionSpeed = sdJoystick.getY();
    double horizontalDirectionSpeed = sdJoystick.getX();
    double rotationalSpeed = sdJoystick.getTwist();
    if(Math.abs(rotationalSpeed) < (0.75) ) {
        rotationalSpeed = 0;
    }

    //x joystick deadzone
    if(Math.abs(horizontalDirectionSpeed) < (0.2) ) {
        horizontalDirectionSpeed = 0;
    }

    //y joystick deadzone
    if(Math.abs(verticalDirectionSpeed) < (0.2)) {
        verticalDirectionSpeed = 0;
    }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed);
        swerveDriveSubsystem.setDesiredChassisSpeeds(desiredSpeeds);

    // switch(selectedDriveMode){
    //   case ("Field Oriented"):
    //     //put field oriented drive here.
    //     // desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed, swerveDriveSubsystem.getPose2d().getRotation());
    //     break;
    //   case ("Robot Oriented"):
    //     //put robot oriented drive here.

    //     ChassisSpeeds desiredSpeeds = new ChassisSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed);
    //     swerveDriveSubsystem.setDesiredChassisSpeeds(desiredSpeeds);
    //     break;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
