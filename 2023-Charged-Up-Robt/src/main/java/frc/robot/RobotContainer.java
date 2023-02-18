// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Operation;
import frc.robot.commands.ControlBucket;
import frc.robot.commands.LineUpWithStation;
import frc.robot.commands.SelfBalance;
import frc.robot.commands.SwerveDriveWithJoystick;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  public final SwerveDriveSubsystem _SwerveDrivebase = Constants.Drivebase.swerveDriveSubsystem;
  public static Joystick _joystick = new Joystick(0);
  public final BucketSubsystem _BucketSubsystem = new BucketSubsystem(Constants.Drivebase.bucketMotorID, _joystick);

  final static JoystickButton activateAutoBalanceButton = new JoystickButton(_joystick, 12);
  final static JoystickButton lineUpWithConeSpotButton = new JoystickButton(_joystick, 8);
  final static JoystickButton bucketUpButton = new JoystickButton(_joystick, 5);
  final static JoystickButton bucketDownButton = new JoystickButton(_joystick, 3);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Operation.kDriverControllerPort);



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
    new Trigger(activateAutoBalanceButton).whileTrue(new SelfBalance(_SwerveDrivebase));
    // new Trigger(bucketUpButton).whileTrue(new ControlBucket(_BucketSubsystem, -0.2));
    // new Trigger(bucketDownButton).whileTrue(new ControlBucket(_BucketSubsystem, -0.1));

    new Trigger(lineUpWithConeSpotButton).onTrue(Commands.run(() -> Autos.LineUpWithConeArea(_SwerveDrivebase), _SwerveDrivebase));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    for(int i = 1; i < 10; i++) {
      SmartDashboard.putData("Drive to " + i, new LineUpWithStation(_SwerveDrivebase, i));
    }
  }
}

//﻿﻿﻿﻿﻿﻿ Pose2d(Translation2d(X: 14.79, Y: 1.06), Rotation2d(Rads: -0.00, Deg: -0.00)) ﻿
//﻿﻿﻿﻿﻿﻿ Translation2d(X: 14.34, Y: 0.23) ﻿

