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

public class DriveXDistanceForward extends CommandBase {
  /** Creates a new DriveToPlace. */

  // inits pid controllers
  final static PIDController xController = new PIDController(6.5, 0, 0);
  final static PIDController yController = new PIDController(6.5, 0, 0);
  final static ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0,
      new TrapezoidProfile.Constraints(5 / 4, 3));

  SwerveDriveSubsystem subsystem;
  Pose2d destination;
  Command moveCommand;
  double distanceX;
  double distanceY;

  // constructor which takes in a distance x and distance y offsetted from
  // starting pose
  public DriveXDistanceForward(SwerveDriveSubsystem subsystem, double distanceX, double distanceY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.distanceX = distanceX;
    this.distanceY = distanceY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //gets the current robot position found using april tags and odometry
    Pose2d currentPose = subsystem.getPose2d();

    // adds x and y offset from pose2d
    this.destination = new Pose2d(new Translation2d(currentPose.getX() + distanceX, currentPose.getY() + distanceY),
        currentPose.getRotation());
    // An ExampleCommand will run in autonomous

    // genrates the trajectory
    // same code as in drivetoplace
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPose,
        // list of no way points
        List.of(),

        Constants.Drivebase.sideifyPose2d(destination),
        trajectoryConfig);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // generates the wrapping command used to run the automatic drive distance
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
