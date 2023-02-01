// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveWithJoystick;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.List;

import org.ejml.equation.Sequence;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveDriveSubsystem _SwerveDrivebase = Constants.DrivebaseConstants.swerveDriveSubsystem;
  public static Joystick _joystick = new Joystick(0);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    _SwerveDrivebase.setDefaultCommand(new SwerveDriveWithJoystick(_SwerveDrivebase, _joystick));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Fligh t
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  Pose2d robotStartPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0.7, 0.5).setKinematics(_SwerveDrivebase._kinematics);
      //4.5, 3.5
      // int rand = (int)(Math.random() * (360) + 1);
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //negative translation2d is right of opening
        robotStartPosition,
        List.of(),
        new Pose2d(-1, Units.inchesToMeters(0), Rotation2d.fromDegrees(0)),
          // new Pose2d(Units.inchesToMeters(320), Units.inchesToMeters(15), Rotation2d.fromDegrees(90)),
          // List.of(
            
        //         new Translation2d(1, 0),
        //         new Translation2d(1, -1),
        //         new Translation2d(0, -1)),
        
        trajectoryConfig);

        PIDController xController = new PIDController(6.5, 0, 0);
        PIDController yController = new PIDController(6.5, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                3, 0, 0, new TrapezoidProfile.Constraints(5/4,3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          _SwerveDrivebase::getPose2d,
          Constants.DrivebaseConstants.kinematics,
          xController,
          yController,
          thetaController,
          _SwerveDrivebase::setModuleStates,
          _SwerveDrivebase);

        return swerveControllerCommand;
      }
    }
