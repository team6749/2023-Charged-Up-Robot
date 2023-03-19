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
  // inits variables
  public SendableChooser<String> orientation = new SendableChooser<String>();
  private String selectedDriveMode;
  private SwerveDriveSubsystem subsystem;
  private Joystick joystick;
  public ChassisSpeeds desiredSpeeds;

  public SwerveDriveWithJoystick(SwerveDriveSubsystem subsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.joystick = joystick;
    orientation.setDefaultOption("Robot Oriented", "Robot Oriented");
    orientation.addOption("Field Oriented", "Field Oriented");
    SmartDashboard.putData(orientation);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // selects drive mode from shuffleboard
    selectedDriveMode = orientation.getSelected();

    // twist rotation and deadzone
    double joystickRotation = joystick.getTwist();
    if (Math.abs(joystickRotation) < (0.30)) {
      joystickRotation = 0;
    }

    double verticalDirectionSpeed = limitedJoystickInput(-joystick.getY()) * 2;
    double horizontalDirectionSpeed = limitedJoystickInput(-joystick.getX()) * 2;
    double rotationalSpeed = limitedJoystickInput(-joystickRotation) * 3;

    if (joystick.getMagnitude() < 0.15) {
      horizontalDirectionSpeed = 0;
      verticalDirectionSpeed = 0;
    }

    // switchcase based on drivemode
    switch (selectedDriveMode) {
      case ("Robot Oriented"):
        // put robot oriented drive here.
        desiredSpeeds = new ChassisSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed, rotationalSpeed);
        break;
      case ("Field Oriented"):
        // fo drive based on alliance color
        if (DriverStation.getAlliance() != Alliance.Red) {
          desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(verticalDirectionSpeed, horizontalDirectionSpeed,
              rotationalSpeed, subsystem.getPose2d().getRotation());
        } else {
          desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-verticalDirectionSpeed, -horizontalDirectionSpeed,
              rotationalSpeed, subsystem.getPose2d().getRotation());
        }
        break;
    }
    // if there is no input on joystick, state is set to straight wheels
    // otherwise, set to desired speeds
    if (verticalDirectionSpeed == 0 && horizontalDirectionSpeed == 0 && rotationalSpeed == 0) {
      subsystem.setModuleStates(new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      });
    } else {
      subsystem.setDesiredChassisSpeeds(desiredSpeeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // limiting joystick sense
  public double limitedJoystickInput(double input) {
    // joystick curve
    // C is the constant multiplier in the limited output eq
    double C = 0.8;
    double limitedOutput = (C * (Math.pow(input, 3))) + ((1 - C) * input);
    return limitedOutput;
  }

}