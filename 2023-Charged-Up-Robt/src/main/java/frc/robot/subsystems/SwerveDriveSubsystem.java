package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDriveSubsystem extends SubsystemBase{
    //priv variables
    public SwerveDriveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    // constructor
    /**
     * @param modules - An Array of SwerveDriveModules
     * Creates a new SwerveDrivebase
     */
    public SwerveDriveSubsystem(SwerveDriveModule[] modules){
        this.modules = modules;
        //constructs a drivebase using translation2ds assigned to each module
        Translation2d[] translations = new Translation2d[modules.length];
        for(int i = 0; i < translations.length; i++){
            translations[i] = modules[i].position;
        }
        _kinematics = new SwerveDriveKinematics(translations);
    }

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
}