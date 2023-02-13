// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LineUpWithStation extends CommandBase {
  private final SwerveDriveSubsystem subsystem;
  private final int substation;
  private final Translation2d stationZero = new Translation2d(1.75, 0.5);
  private final Translation2d spacing = new Translation2d(0, 0.5588);

  private Command moveCommand;

  /** Creates a new LineUpWithStation. */
  public LineUpWithStation(SwerveDriveSubsystem subsystem, int station) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.substation = station;
    this.subsystem = subsystem;
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = subsystem.getPose2d();

    Pose2d targetPosition = new Pose2d(stationZero.plus(spacing.times(substation - 1)), Rotation2d.fromDegrees(180));
    double yOffset = (currentPose.getY() - targetPosition.getY()) / 2;

    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPose,
        List.of(
            Constants.Drivebase.sideifyTranslation2d(new Translation2d(2.2, currentPose.getY())),
            Constants.Drivebase.sideifyTranslation2d(new Translation2d(2.2, yOffset + targetPosition.getY()))),
        Constants.Drivebase.sideifyPose2d(targetPosition),
        trajectoryConfig);

    PIDController xController = new PIDController(6.5, 0, 0);
    PIDController yController = new PIDController(6.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        3, 0, 0, new TrapezoidProfile.Constraints(5 / 4, 3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    moveCommand = new SwerveControllerCommand(
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
