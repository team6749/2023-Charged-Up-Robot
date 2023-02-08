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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
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
    PathPlannerTrajectory BlueAutoBalanceOnlyPath = PathPlanner.loadPath("killme", new PathConstraints(1.5, 1.5));
    return subsystem.followTrajectoryCommand(BlueAutoBalanceOnlyPath, true);
  }



}
