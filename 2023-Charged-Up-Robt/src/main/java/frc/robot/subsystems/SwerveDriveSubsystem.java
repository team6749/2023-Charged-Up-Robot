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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDriveSubsystem extends SubsystemBase{
    //priv variables
    public SwerveDriveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    // public SwerveDriveOdometry odometry;

    // constructor
    /**
     * @param modules - An Array of SwerveDriveModules
     * Creates a new SwerveDrivebase
     */
    public SwerveDriveSubsystem(SwerveDriveModule[] modules){
        this.modules = modules;
        //constructs a drivebase using translation2ds assigned to each module
        Translation2d[] translations = new Translation2d[modules.length];
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < translations.length; i++){
            translations[i] = modules[i].position;
        }
        // TODO - FIX LATER
    _kinematics = new SwerveDriveKinematics(translations);
        // SwerveDriveOdometry odometry = new SwerveDriveOdometry(_kinematics, gyro.getRotation2d(), new SwerveModulePosition[]{
        //     modules[0].position,
        //     modules[1].position,
        //     modules[2].position,
        //     modules[3].position
        // });
    }
    private Rotation2d getChassisRotation(){
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
    // public Pose2d getPose2d(){
    //     return odometry.getPoseMeters();
    // }
    //TODO
    // public void resetOdometry(Pose2d pose) {
    //     odometry.resetPosition(getChassisRotation(), x, pose);
    // }

    @Override
    public void periodic() {
        //Call periodic on children
        for (SwerveDriveModule swerveModule : modules) {
            swerveModule.periodic();
        }


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

    public void setDesiredChassisSpeeds(ChassisSpeeds cSpeeds, Translation2d pivotpoint) {
        SwerveModuleState[] _desiredStates = _kinematics.toSwerveModuleStates(cSpeeds,pivotpoint);
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

}