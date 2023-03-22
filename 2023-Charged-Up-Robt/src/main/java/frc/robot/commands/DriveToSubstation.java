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

public class DriveToSubstation extends CommandBase {

  private SwerveDriveSubsystem subsystem;
  private Command moveCommand;
  boolean m_leftStation;

  /** Creates a new IntakeFromSubstation. */
  public DriveToSubstation(SwerveDriveSubsystem subsystem, boolean station) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_leftStation = station;
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = subsystem.getPose2d();

    Pose2d leftSubstationPose = Constants.Drivebase.sideifyPose2d(new Pose2d(15.8, 7.33, Rotation2d.fromDegrees(0)));
    Pose2d rightSubstationPose = Constants.Drivebase.sideifyPose2d(new Pose2d(15.8, 6.0, Rotation2d.fromDegrees(0)));

    // An ExampleCommand will run in autonomous
    // trajectory generator
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2).setKinematics(subsystem._kinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        currentPose,
        List.of(),
        m_leftStation ? leftSubstationPose : rightSubstationPose,
        trajectoryConfig);

    // pid controllers used to correct any error
    PIDController xController = new PIDController(6.5, 0, 0);
    PIDController yController = new PIDController(6.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        3, 0, 0, new TrapezoidProfile.Constraints(5 / 4, 3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // command to wrap trajectory generator
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
