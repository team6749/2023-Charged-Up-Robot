// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DrivebaseConstants {
    //subsystem constructed with array of modules
    //62.726cm
    //31.363cm
    public static double halfWidth = 0.25; // in meters
    public static double halfHeight = 0.185; // in meters

    public static Translation2d frontLeftPosition = new Translation2d(+halfHeight, +halfWidth);
    public static Translation2d frontRightPosition = new Translation2d(+halfHeight, -halfWidth);
    public static Translation2d backLeftPosition = new Translation2d(-halfHeight, +halfWidth);
    public static Translation2d backRightPosition = new Translation2d(-halfHeight, -halfWidth);
    
    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.DrivebaseConstants.frontLeftPosition, 
      Constants.DrivebaseConstants.frontRightPosition, 
      Constants.DrivebaseConstants.backLeftPosition, 
      Constants.DrivebaseConstants.backRightPosition
    );
    
    public static SwerveDriveModule frontLeftModule = new SwerveDriveModule(
      "frontLeftModule",
      4,
      5, 
      6, 
      105.5, 
      frontLeftPosition
    );
    public static SwerveDriveModule frontRightModule = new SwerveDriveModule(
      "frontRightModule", 
      7, 
      8, 
      9, 
      -235.1, 
      frontRightPosition
    );
    public static SwerveDriveModule backLeftModule = new SwerveDriveModule(
      "backLeftModule", 
      1, 
      2, 
      3, 
      145.3, 
      backLeftPosition
    );
    public static SwerveDriveModule backRightModule = new SwerveDriveModule(
      "backRightModule", 
      10, 
      11, 
      12, 
      -68.1, 
      backRightPosition
    );
    



    public static SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveDriveModule [] {frontLeftModule, frontRightModule, backLeftModule, backRightModule});

    
  }
}
