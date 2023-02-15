// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SelfBalance;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {
    /** Example static factory for an autonomous command. */
//   public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
//     return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
//   }
  final static PIDController xController = new PIDController(6.5, 0, 0);
  final static PIDController yController = new PIDController(6.5, 0, 0);
  final static ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(5/4,3));
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  
  //path planner lib auto mirrors / flips paths
  //starts closer to charge station
  public static CommandBase ChargingStationOnlyTop(SwerveDriveSubsystem subsystem) {
    PathPlannerTrajectory ChargingStationOnlyTop = PathPlanner.loadPath("ChargingStationOnlyTop", new PathConstraints(1.5, 1.5));
    return subsystem.followTrajectoryCommand(ChargingStationOnlyTop, true).andThen(new SelfBalance(subsystem));

  }
  
  //path planner lib auto mirrors / flips paths
  //closer to gates
  public static CommandBase ChargingStationOnlyBottom(SwerveDriveSubsystem subsystem) {
    PathPlannerTrajectory ChargingStationOnlyBottom = PathPlanner.loadPath("ChargingStationOnlyBottom", new PathConstraints(1.5, 1.5));
    return subsystem.followTrajectoryCommand(ChargingStationOnlyBottom, true).andThen(new SelfBalance(subsystem));
  }

  //literally do nothing at all
  public static CommandBase doNothing (SwerveDriveSubsystem subsystem) {
    return new CommandBase() {};
  }

  //pplib command to drive straight forward 2 meters
  public static CommandBase driveForward (SwerveDriveSubsystem subsystem){
    Pose2d currentPose = subsystem.getPose2d();

    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      currentPose,
      List.of(),
      // uses start position and moves 2m forward on x
      new Pose2d(currentPose.getTranslation().plus(new Translation2d(2, 0)), currentPose.getRotation()),
       trajectoryConfig);
       thetaController.enableContinuousInput(-Math.PI, Math.PI);

       
      return new SwerveControllerCommand(
        trajectory,
        subsystem::getPose2d,
        subsystem._kinematics,
        xController,
        yController,
        thetaController,
        subsystem::setModuleStates,
        subsystem);
  }

  //drive forward 1.3m and balance
  //START 0.3m FROM CHARGING PAD RAMP
  public static CommandBase forwardAndBalance (SwerveDriveSubsystem subsystem){
    Pose2d currentPose = subsystem.getPose2d();

    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      currentPose,
      List.of(),

      new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.3, 0)), currentPose.getRotation()),
       trajectoryConfig);
      
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      return new SwerveControllerCommand(
        trajectory,
        subsystem::getPose2d,
        subsystem._kinematics,
        xController,
        yController,
        thetaController,
        subsystem::setModuleStates,
        subsystem);

  }

  //custom command w/o pplib
  public static Command LineUpWithConeArea(SwerveDriveSubsystem subsystem){
    Pose2d currentPose = subsystem.getPose2d();

    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      currentPose,
      List.of(),
      new Pose2d(1.8, 1.6, Rotation2d.fromDegrees(180)),
       trajectoryConfig);
      
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      return new SwerveControllerCommand(
        trajectory,
        subsystem::getPose2d,
        subsystem._kinematics,
        xController,
        yController,
        thetaController,
        subsystem::setModuleStates,
        subsystem);
      // return new WaitCommand(1);
  }



}
