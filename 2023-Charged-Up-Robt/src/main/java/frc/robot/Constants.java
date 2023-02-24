
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
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
  public static class Operation {

    public static final int kDriverControllerPort = 0;

  }

  public static class Arm{
    public static int armBaseMotor;
    public static int armMiddleMotor;
    public static int armClawMotor;

    public static int armBaseEncoder = 1;
    public static int armMiddleEncoder = 2;
    public static int armClawEncoder = 3;
  }
  public static class Drivebase {
    //subsystem constructed with array of modules
    //62.726cm
    //31.363cm
    public static double halfWidth = 0.185; // in meters
    public static double halfHeight = 0.25; // in meters
    public static double wheelDiameter = 0.097; //meters
    public static Translation2d frontLeftPosition = new Translation2d(+halfHeight, +halfWidth);
    public static Translation2d frontRightPosition = new Translation2d(+halfHeight, -halfWidth);
    public static Translation2d backLeftPosition = new Translation2d(-halfHeight, +halfWidth);
    public static Translation2d backRightPosition = new Translation2d(-halfHeight, -halfWidth);
    
    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.Drivebase.frontLeftPosition, 
      Constants.Drivebase.frontRightPosition, 
      Constants.Drivebase.backLeftPosition, 
      Constants.Drivebase.backRightPosition
    );
    
    public static int bucketMotorID = 20;
    public static SwerveDriveModule backRightModule = new SwerveDriveModule(
      "frontLeftModule",
      4,
      5, 
      6, 
      103.5, 
      backRightPosition
    );
    public static SwerveDriveModule backLeftModule = new SwerveDriveModule(
      "frontRightModule", 
      7, 
      8, 
      9, 
      -236.6, 
      backLeftPosition
    );
    public static SwerveDriveModule frontRightModule = new SwerveDriveModule(
      "backLeftModule", 
      1, 
      2, 
      3, 
      140.0, 
      frontRightPosition
    );
    public static SwerveDriveModule frontLeftModule = new SwerveDriveModule(
      "backRightModule", 
      10, 
      11, 
      12, 
      -66.6, 
      frontLeftPosition
    );
    



    public static SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveDriveModule [] {frontLeftModule, frontRightModule, backLeftModule, backRightModule});


    public static final double fieldLengthInMeters = 16.54;
    public static final double fieldWidthInMeters = 8.02;
    
    //everything is blue sided
    //sideify function
    public static Pose2d sideifyPose2d(Pose2d blue){

      if(DriverStation.getAlliance() != DriverStation.Alliance.Red){
        //If we are not red (aka blue) return the blue value
        return blue;
      }

      Rotation2d redSideRotation = blue.getRotation().plus(Rotation2d.fromDegrees(180));
      
      return new Pose2d(sideifyTranslation2d(blue.getTranslation()), redSideRotation);
    }

    public static Translation2d sideifyTranslation2d(Translation2d blue){
      if(DriverStation.getAlliance() != DriverStation.Alliance.Red){
        //If we are not red (aka blue) return the blue value
        return blue;
      }

      double redSideLength = fieldLengthInMeters - blue.getX();
      
      return new Translation2d(redSideLength, blue.getY());
    }
  }
}
