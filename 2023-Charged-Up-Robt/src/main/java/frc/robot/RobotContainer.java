// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClawToggle;
import frc.robot.commands.DoAutoAlignmentAndScore;
import frc.robot.commands.DriveToSubstation;
import frc.robot.commands.LineUpWithStation;
import frc.robot.commands.SelfBalance;
import frc.robot.commands.SwerveDriveWithController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.commands.MoveArmSegmentManually;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveDriveSubsystem _SwerveDrivebase = Constants.Drivebase.swerveDriveSubsystem;

  // public static Joystick _joystick = new Joystick(0);
  public static XboxController _armController = new XboxController(1);
  public static XboxController _controller = new XboxController(0);
  
  // public final BucketSubsystem _BucketSubsystem = new BucketSubsystem(Constants.Drivebase.bucketMotorID, _joystick);
  public final ArmSubsystem _ArmSubsystem = new ArmSubsystem();
  public final ClawSubsystem _ClawSubsystem = new ClawSubsystem();
  public final int one = 1;


  // final static JoystickButton activateAutoBalanceButton = new JoystickButton(_joystick, 12);
  // final static JoystickButton lineUpWithConeSpotButton = new JoystickButton(_joystick, 8);
  // final static JoystickButton bucketUpButton = new JoystickButton(_joystick, 5);
  // final static JoystickButton bucketDownButton = new JoystickButton(_joystick, 3);
  //  final static JoystickButton moveArmUpButton = new JoystickButton(_joystick, 9);
  // final static JoystickButton moveArmDownButton = new JoystickButton(_joystick, 10);


  //main driver controller triggers
  final static JoystickButton activateAutoBalanceButton = new JoystickButton(_controller, 7); //back

  final static Trigger rightSubstation = new Trigger(() -> _controller.getPOV() == 90); //dpad right
  final static Trigger leftSubstation = new Trigger(() -> _controller.getPOV() == 270); //dpad left
  final static Trigger ground = new Trigger(() -> _controller.getPOV() == 180); //dpad down
  final static Trigger idle = new Trigger(() -> _controller.getPOV() == 0); //dpad up

  final static JoystickButton scoreMiddle = new JoystickButton(_controller, 4);
  final static JoystickButton scoreBottom = new JoystickButton(_controller, 1);
  
  final static JoystickButton intakeSubstation = new JoystickButton(_controller, 2);

  final static JoystickButton autoAlign = new JoystickButton(_controller, 8);

  final static Trigger moveArmUp = new Trigger(() -> _controller.getRightTriggerAxis() > 0.5); //back right trigger
  final static Trigger moveClawUp = new Trigger(() -> _controller.getLeftTriggerAxis() > 0.5); //back left trigger

  final static JoystickButton moveArmDown = new JoystickButton(_controller, 6); //left bumber
  final static JoystickButton moveClawDown = new JoystickButton(_controller, 5); //right bumper

  final static JoystickButton toggleClawButton = new JoystickButton(_controller, 3); //a button on controller


  


  //arm controller triggers
  final static Trigger rightSubstation2 = new Trigger(() -> _armController.getPOV() == 90); //dpad right
  final static Trigger leftSubstation2 = new Trigger(() -> _armController.getPOV() == 270); //dpad left
  final static Trigger ground2 = new Trigger(() -> _armController.getPOV() == 180); //dpad down
  final static Trigger idle2 = new Trigger(() -> _armController.getPOV() == 0); //dpad up

  final static JoystickButton scoreMiddle2 = new JoystickButton(_armController, 4);
  final static JoystickButton scoreBottom2 = new JoystickButton(_armController, 1);
  
  final static JoystickButton intakeSubstation2 = new JoystickButton(_armController, 2);

  final static Trigger moveArmUp2 = new Trigger(() -> _armController.getRightTriggerAxis() > 0.5); //back right trigger
  final static Trigger moveClawUp2 = new Trigger(() -> _armController.getLeftTriggerAxis() > 0.5); //back left trigger

  final static JoystickButton moveArmDown2 = new JoystickButton(_armController, 6); //left bumber
  final static JoystickButton moveClawDown2 = new JoystickButton(_armController, 5); //right bumper

  final static JoystickButton toggleClawButton2 = new JoystickButton(_armController, 3); //a button on controller


  public SendableChooser<Command> scoringSelector = new SendableChooser<Command>();
  public SendableChooser<Command> stationSelector = new SendableChooser<Command>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController = new CommandXboxController(Operation.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    scoringSelector.setDefaultOption("Middle", Autos.PlaceMiddle(_ArmSubsystem, _ClawSubsystem));
    scoringSelector.addOption("Low", Autos.PlaceBottom(_ArmSubsystem, _ClawSubsystem));
    scoringSelector.addOption("Dont Score", Autos.doNothing(_SwerveDrivebase));
    SmartDashboard.putData("Scoring Selector:", scoringSelector);
    //this code adds the stations 1-9 to the sendable chooser `stationSelector` 
    for (int i = 1; i < 10; i++) {
      stationSelector.addOption("Station " + i + " align", new LineUpWithStation(_SwerveDrivebase, i));
    }
    //then we would add something that triggers it to go to the selected station
    SmartDashboard.putData("Station Selected:", stationSelector);


    // Configure the trigger bindings
    configureBindings();
    
    _SwerveDrivebase.setDefaultCommand(new SwerveDriveWithController(_SwerveDrivebase, _controller));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Fligh t
   * joysticks}.
   */

  // public CommandBase setArmAndLineUp() {
  //   return new SequentialCommandGroup(
  //     new MoveArmSegment(_ArmSubsystem.baseSegment, 45),
  //     new MoveArmSegment(_ArmSubsystem.clawSegment, 45),
  //     new LineUpWithStation(_SwerveDrivebase, 1)
  //   );
  // }

  private void configureBindings() {


    //controller one
    new Trigger(activateAutoBalanceButton).whileTrue(new SelfBalance(_SwerveDrivebase));

    // new Trigger(lineUpWithConeSpotButton)
    //     .onTrue(Commands.run(() -> Autos.LineUpWithConeArea(_SwerveDrivebase), _SwerveDrivebase));

    new Trigger(moveClawUp).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.clawSegment, 0.2));
    new Trigger(moveClawDown).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.clawSegment, -0.2));

    new Trigger(moveArmUp).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.baseSegment, 0.2));
    new Trigger(moveArmDown).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.baseSegment, -0.2));

    
    new Trigger(rightSubstation).whileTrue(Constants.ArmCommands.moveArmIdle(_ArmSubsystem)
        .andThen(new DriveToSubstation(_SwerveDrivebase, false))
        .andThen(Autos.GrabFromSubation(_ArmSubsystem, _ClawSubsystem)));

    new Trigger(leftSubstation).whileTrue(Constants.ArmCommands.moveArmIdle(_ArmSubsystem)
        .andThen(new DriveToSubstation(_SwerveDrivebase, true))
        .andThen(Autos.GrabFromSubation(_ArmSubsystem, _ClawSubsystem)));

    new Trigger(ground).whileTrue(Constants.ArmCommands.MoveArmToGround(_ArmSubsystem));
    new Trigger(idle).whileTrue(Constants.ArmCommands.moveArmIdle(_ArmSubsystem));

    new Trigger(scoreMiddle).whileTrue(Autos.PlaceMiddle(_ArmSubsystem, _ClawSubsystem));
    new Trigger(scoreBottom).whileTrue(Autos.PlaceBottom(_ArmSubsystem, _ClawSubsystem));

    new Trigger(intakeSubstation).whileTrue(Autos.GrabFromSubation(_ArmSubsystem, _ClawSubsystem));

    new Trigger(autoAlign).whileTrue( new DoAutoAlignmentAndScore(this) );



    new Trigger(toggleClawButton).whileTrue(new ClawToggle(_ClawSubsystem));



    //controller 2 
    new Trigger(moveClawUp2).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.clawSegment, 0.2));
    new Trigger(moveClawDown2).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.clawSegment, -0.2));

    new Trigger(moveArmUp2).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.baseSegment, 0.2));
    new Trigger(moveArmDown2).whileTrue(new MoveArmSegmentManually(_ArmSubsystem.baseSegment, -0.2));

    
    new Trigger(rightSubstation2).whileTrue(Constants.ArmCommands.moveArmIdle(_ArmSubsystem)
        .andThen(new DriveToSubstation(_SwerveDrivebase, false))
        .andThen(Autos.GrabFromSubation(_ArmSubsystem, _ClawSubsystem)));

    new Trigger(leftSubstation2).whileTrue(Constants.ArmCommands.moveArmIdle(_ArmSubsystem)
        .andThen(new DriveToSubstation(_SwerveDrivebase, true))
        .andThen(Autos.GrabFromSubation(_ArmSubsystem, _ClawSubsystem)));

    new Trigger(ground2).whileTrue(Constants.ArmCommands.MoveArmToGround(_ArmSubsystem));
    new Trigger(idle2).whileTrue(Constants.ArmCommands.moveArmIdle(_ArmSubsystem));

    new Trigger(scoreMiddle2).whileTrue(Autos.PlaceMiddle(_ArmSubsystem, _ClawSubsystem));
    new Trigger(scoreBottom2).whileTrue(Autos.PlaceBottom(_ArmSubsystem, _ClawSubsystem));

    new Trigger(intakeSubstation2).whileTrue(Autos.GrabFromSubation(_ArmSubsystem, _ClawSubsystem));

    new Trigger(toggleClawButton2).whileTrue(new ClawToggle(_ClawSubsystem));
    
  }


}
