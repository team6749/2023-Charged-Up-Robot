package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveDriveSubsystem extends SubsystemBase{
    //priv variables
    public SwerveDriveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    public SwerveDriveOdometry odometry;

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
        gyro.getRotation2d(), 
        getCurrentModulePositions()
    );
    
}


    @Override
    public void periodic() {
        //Call periodic on children
        for (SwerveDriveModule swerveModule : modules) {
            swerveModule.periodic();
        }
        
        odometry.update(
            getRotation(), 
            getCurrentModulePositions()
            );

        SmartDashboard.putNumber("Swerve Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Swerve Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Swerve Odometry Rot", odometry.getPoseMeters().getRotation().getDegrees());
        }

        
    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(gyro.getAngle());
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
}