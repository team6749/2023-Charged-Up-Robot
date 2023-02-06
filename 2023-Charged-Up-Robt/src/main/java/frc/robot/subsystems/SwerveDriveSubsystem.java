package frc.robot.subsystems;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class SwerveDriveSubsystem extends SubsystemBase{
    //priv variables
    public static ADIS16470_IMU gyro = new ADIS16470_IMU();
    public SwerveDriveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    // public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    public BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    public SwerveDriveOdometry odometry;
    HashMap<String, Command> eventMap = new HashMap<>();
    // constructor
    /**
     * @param modules - An Array of SwerveDriveModules
     * Creates a new SwerveDrivebase
     */
    public SwerveDriveSubsystem(SwerveDriveModule[] modules){
        this.modules = modules;
        _kinematics = Constants.DrivebaseConstants.kinematics;

    odometry = new SwerveDriveOdometry(
        _kinematics,
        getRotation(), 
        getCurrentModulePositions()
    );
    
}


    @Override
    public void periodic() {
        //Call periodic on children
        for (SwerveDriveModule swerveModule : modules) {
            swerveModule.periodic();
        }
        // System.out.println(gyro.getYComplementaryAngle());
        odometry.update(
            getRotation(), 
            getCurrentModulePositions()
            );

        SmartDashboard.putNumber("Swerve Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Swerve Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Swerve Odosmetry Rot", odometry.getPoseMeters().getRotation().getDegrees());

        // SmartDashboard.putNumber("acc x", gyro2.getYComplementaryAngle());
        // SmartDashboard.putNumber("acc y", gyro2.getYFilteredAccelAngle());
        // SmartDashboard.putNumber("angle", gyro2.getAngle());
        // SmartDashboard.putNumber("acc z", accelerometer.getZ());
        }

        
    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation(), 
            getCurrentModulePositions(), 
            pose
        );
    }

    
    /**
     * @param cSpeeds the desired chassis speeds to set each module to
     * converts each module to the ChassisSpeeds cSpeeds
     */
    public void setDesiredChassisSpeeds(ChassisSpeeds cSpeeds) {
        SwerveModuleState[] _desiredStates = _kinematics.toSwerveModuleStates(cSpeeds);
        for (int i = 0; i < _desiredStates.length; i++) {
            modules[i].setDesiredState(_desiredStates[i]);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < desiredStates.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }        
    }

    public SwerveModulePosition[] getCurrentModulePositions(){
        return new SwerveModulePosition[]{
            modules[0].getCurrentPosition(),
            modules[1].getCurrentPosition(),
            modules[2].getCurrentPosition(),
            modules[3].getCurrentPosition()
        };
    }


    //creates a path to follow using the parameter trjactoery and returns the auto command
// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public CommandBase followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose2d, // Pose supplier
             this._kinematics, // SwerveDriveKinematics
             new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
             this // Requires this drive subsystem
         )
     );
 }
}