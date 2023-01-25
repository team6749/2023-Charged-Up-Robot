// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    //9.75 inches
    public static SwerveDriveModule frontLeftModule = new SwerveDriveModule("frontLeftModule", 1, 2 , 3, 0, new Translation2d(Units.inchesToMeters(-9.75), Units.inchesToMeters(9.75)));

    public static SwerveDriveModule frontRightModule = new SwerveDriveModule("frontRightModule", 4, 5, 6, 0, new Translation2d(Units.inchesToMeters(9.75), Units.inchesToMeters(9.75)));

    public static SwerveDriveModule backRightModule = new SwerveDriveModule("backRightModule", 7, 8, 9, 0, new Translation2d(Units.inchesToMeters(9.75), Units.inchesToMeters(-9.75)));

    public static SwerveDriveModule backLeftModule = new SwerveDriveModule("backLeftModule", 10, 11, 12, 0, new Translation2d(Units.inchesToMeters(-9.75), Units.inchesToMeters(-9.75)));
    
    public static SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveDriveModule [] {frontLeftModule, frontRightModule, backRightModule, backLeftModule});
  }
}
