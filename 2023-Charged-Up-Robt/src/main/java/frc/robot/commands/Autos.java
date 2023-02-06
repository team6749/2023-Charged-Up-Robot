// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.List;

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

    

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1.5, 1.5).setKinematics(subsystem._kinematics);
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //negative translation2d is right of opening
        subsystem.getPose2d(),
        List.of(),
        //TODO replace this with the field corrdinates of the charging station center
        new Pose2d(1.9, Units.inchesToMeters(0), Rotation2d.fromDegrees(0)),
        trajectoryConfig);

    return new SequentialCommandGroup(
        subsystem.drivePath(trajectory),
        new SelfBalance(subsystem)
    );
  }



}
