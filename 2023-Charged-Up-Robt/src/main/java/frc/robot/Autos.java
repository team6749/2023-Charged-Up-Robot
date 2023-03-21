// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClawControl;
import frc.robot.commands.DriveXDistanceForward;
import frc.robot.commands.SelfBalance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
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

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }
  final static PIDController xController = new PIDController(6.5, 0, 0);
  final static PIDController yController = new PIDController(6.5, 0, 0);
  final static ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0,
      new TrapezoidProfile.Constraints(5 / 4, 3));

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  // literally do nothing at all
  public static CommandBase doNothing(SwerveDriveSubsystem subsystem) {
    return new CommandBase() {
    };
  }

  // pplib command to drive straight forward 2 meters
  public static CommandBase driveForward(SwerveDriveSubsystem subsystem) {
    return new DriveXDistanceForward(subsystem, 2, 0);
  }

  // drive forward 1.3m and balance
  // START 0.3m FROM CHARGING PAD RAMP
  public static Command ForwardAndBalance(SwerveDriveSubsystem subsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    return PlaceMiddle(armSubsystem, clawSubsystem)
        .andThen(new DriveXDistanceForward(subsystem, -2.3, 0)
        .andThen(new SelfBalance(subsystem)));
  }

  // place and do nothing
  public static Command PlaceMiddle(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    return Constants.ArmCommands.MoveArmToMiddle(armSubsystem)
        .andThen(new ClawControl(clawSubsystem, true))
        .andThen(Constants.ArmCommands.moveArmIdle(armSubsystem))
        .andThen(new ClawControl(clawSubsystem, false));
  }

  // place and leave community
  public static Command PlaceAndLeaveCommunity(SwerveDriveSubsystem swerveDriveSubsystem, ArmSubsystem armSubsystem,
      ClawSubsystem clawSubsystem) {
    return PlaceMiddle(armSubsystem, clawSubsystem)
        .andThen(new DriveXDistanceForward(swerveDriveSubsystem, -4.1, 0));
  }

  // place cone and balance (lower)
  public static CommandBase LowerPlaceAndBalanceCone(SwerveDriveSubsystem swerveDriveSubsystem,
      ArmSubsystem armSubsystem,
      ClawSubsystem clawSubsystem) {
    PathPlannerTrajectory LowerPlaceAndBalanceCone = PathPlanner.loadPath("LowerPlaceAndBalanceCone",
        new PathConstraints(1.5, 1.5));
    swerveDriveSubsystem.field.getObject("autostart").setPose(LowerPlaceAndBalanceCone.getInitialPose());
    return Constants.ArmCommands.MoveArmToMiddle(armSubsystem)
        .andThen(new ClawControl(clawSubsystem, true))
        .andThen(Constants.ArmCommands.moveArmIdle(armSubsystem))
        .andThen(swerveDriveSubsystem.followTrajectoryCommand(LowerPlaceAndBalanceCone, true))
        .andThen(new ClawControl(clawSubsystem, false))
        .andThen(new SelfBalance(swerveDriveSubsystem));
  }

  // place cone and balance (upper)
  public static CommandBase UpperPlaceAndBalanceCone(SwerveDriveSubsystem swerveDriveSubsystem,
      ArmSubsystem armSubsystem,
      ClawSubsystem clawSubsystem) {
    PathPlannerTrajectory UpperPlaceAndBalanceCone = PathPlanner.loadPath("UpperPlaceAndBalanceCone",
        new PathConstraints(1.5, 1.5));
    swerveDriveSubsystem.field.getObject("autostart").setPose(UpperPlaceAndBalanceCone.getInitialPose());
    return Constants.ArmCommands.MoveArmToMiddle(armSubsystem)
        .andThen(new ClawControl(clawSubsystem, true))
        .andThen(Constants.ArmCommands.moveArmIdle(armSubsystem))
        .andThen(swerveDriveSubsystem.followTrajectoryCommand(UpperPlaceAndBalanceCone, true))
        .andThen(new SelfBalance(swerveDriveSubsystem))
        .andThen(new ClawControl(clawSubsystem, false));
  }

  // custom command w/o pplib
  public static Command LineUpWithConeArea(SwerveDriveSubsystem subsystem) {
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
