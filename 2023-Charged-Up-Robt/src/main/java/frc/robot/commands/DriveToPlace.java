// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToPlace extends CommandBase {
  /** Creates a new DriveToPlace. */

  // inits pid controllers
  final static PIDController xController = new PIDController(6.5, 0, 0);
  final static PIDController yController = new PIDController(6.5, 0, 0);
  final static ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0,
      new TrapezoidProfile.Constraints(5 / 4, 3));

  SwerveDriveSubsystem subsystem;
  List<Translation2d> waypoints;
  Pose2d destination;
  Command moveCommand;

  // constructor without waypoints
  public DriveToPlace(SwerveDriveSubsystem subsystem, Pose2d destination) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);
    waypoints = null;
    this.destination = destination;
  }

  // constructor with waypoints
  public DriveToPlace(SwerveDriveSubsystem subsystem, Pose2d destination, List<Translation2d> waypoints) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);

    // adds each translation from the list to a private list for easier access, AND
    // sideifies each point (in the 2d space)
    for (Translation2d point : waypoints) {

      this.waypoints.add(Constants.Drivebase.sideifyTranslation2d(point));

    }
    this.destination = destination;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // gets the starting robot pose
    Pose2d currentPose = subsystem.getPose2d();

    // An ExampleCommand will run in autonomous
    // generates a trjactory
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(

        currentPose,
        waypoints,
        Constants.Drivebase.sideifyPose2d(destination),

        trajectoryConfig);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // generates a command within the command in order to run with the current robot
    // pose
    // this is called wrapping
    this.moveCommand = new SwerveControllerCommand(
        trajectory,
        subsystem::getPose2d,
        subsystem._kinematics,
        xController,
        yController,
        thetaController,
        subsystem::setModuleStates,
        subsystem);
    moveCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    moveCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    moveCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return moveCommand.isFinished();
  }
}
