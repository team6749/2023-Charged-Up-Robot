// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.math.util.Units;

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
    public static double distanceToCenter = 0.31363; // in meters

    public static Translation2d frontLeftPosition = new Translation2d(+distanceToCenter, +distanceToCenter);
    public static Translation2d frontRightPosition = new Translation2d(+distanceToCenter, -distanceToCenter);
    public static Translation2d backLeftPosition = new Translation2d(-distanceToCenter, +distanceToCenter);
    public static Translation2d backRightPosition = new Translation2d(-distanceToCenter, -distanceToCenter);
    
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

    //define the positions of the april tags on the field and 
    //create the layout of them on the field to update pose in 
    //relation to the tags
    public static AprilTag[] tags = new AprilTag[] {
      new AprilTag(1,
          new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, 3.1415))),
      new AprilTag(2,
          new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, 3.1415))),
      new AprilTag(3, 
          new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, 3.1415))),
      new AprilTag(4, 
          new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
              new Rotation3d(0, 0, 3.1415))),
      new AprilTag(5, 
          new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
              new Rotation3d(0, 0, 0))),
      new AprilTag(6, 
          new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
              new Rotation3d(0, 0, 0))),
      new AprilTag(7, 
              new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
                  new Rotation3d(0, 0, 0))),
      new AprilTag(8, 
               new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
                  new Rotation3d(0, 0, 0)))
            };
  
  public static AprilTagFieldLayout layout2023 = new AprilTagFieldLayout(
      List.of(tags),
      Units.inchesToMeters(615.25),
      Units.inchesToMeters(315.5));   
  
  //define the position of the camera on the robot
  public static Transform3d cameraPosition = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  }
}
