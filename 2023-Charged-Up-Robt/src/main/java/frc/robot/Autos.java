// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
public final class Autos {
    private SwerveDriveSubsystem subsystem;
  /** Example static factory for an autonomous command. */
//   public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
//     return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
//   }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
  

  public static CommandBase BlueAutoBalanceOnlyAuto(SwerveDriveSubsystem subsystem) {
    PathPlannerTrajectory BlueAutoBalanceOnlyPath = PathPlanner.loadPath("todays path", new PathConstraints(1.5, 1.5));
    return subsystem.followTrajectoryCommand(BlueAutoBalanceOnlyPath, true);
  }

  public static Command LineUpWithConeArea(SwerveDriveSubsystem subsystem){
    Pose2d asdf = subsystem.getPose2d();

    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      asdf,
      List.of(),
      new Pose2d(1.8, 1.6, Rotation2d.fromDegrees(180)),
       trajectoryConfig);

      PIDController xController = new PIDController(6.5, 0, 0);
      PIDController yController = new PIDController(6.5, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
              3, 0, 0, new TrapezoidProfile.Constraints(5/4,3));
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
