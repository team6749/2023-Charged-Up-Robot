package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;



public class SwerveDriveSubsystem extends SubsystemBase{
    //priv variables
    public static ADIS16470_IMU gyro = new ADIS16470_IMU();
    public SwerveDriveModule[] modules;
    public SwerveDriveKinematics _kinematics;
    public SwerveModuleState[] states;
    // public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    public BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    public SwerveDriveOdometry odometry;

    public SwerveDrivePoseEstimator poseEstimator;

    //field map for smartdashboard and pose on smartdashboard
    public final Field2d field = new Field2d();

    // initializing grabbing the data from the camera after processing in photon,
    // name the camera in photon vision the same as the camera name string in code
    PhotonCamera camera = new PhotonCamera("robotcamera");

    //define the positions of the april tags on the field and 
    //create the layout of them on the field to update pose in 
    //relation to the tags
    public static AprilTag[] tags = new AprilTag[] {
        new AprilTag(1,
            new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, 3.1415))),
        new AprilTag(2,
            new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, 3.1415))),
        new AprilTag(3, 
            new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, 3.1415))),
        new AprilTag(4, 
            new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
                new Rotation3d(0, 0, 3.1415))),
        new AprilTag(5, 
            new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
                new Rotation3d(0, 0, 0))),
        new AprilTag(6, 
            new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
                new Rotation3d(0, 0, 0))),
        new AprilTag(7, 
                new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
                    new Rotation3d(0, 0, 0))),
        new AprilTag(8, 
                 new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
                    new Rotation3d(0, 0, 0)))
              };
    
    public static AprilTagFieldLayout layout2023 = new AprilTagFieldLayout(
        List.of(tags),
        Units.inchesToMeters(615.25),
        Units.inchesToMeters(315.5));   
    
    //define the position of the camera on the robot
    public static Transform3d cameraPosition = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    
    

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(layout2023, PoseStrategy.AVERAGE_BEST_TARGETS, camera, cameraPosition);



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
    poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getRotation(), getCurrentModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    //add the field map to smartdashboard
    SmartDashboard.putData("field map", field);
}


    @Override
    public void periodic() {
        //VISION STUFF
        
        Optional<EstimatedRobotPose> estPose = photonPoseEstimator.update();
        if(estPose.isPresent()) {
            SmartDashboard.putString("pose est", estPose.get().estimatedPose.toString());
             poseEstimator.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);     
          }

        //Call periodic on children
        for (SwerveDriveModule swerveModule : modules) {
            swerveModule.periodic();
        }
        // System.out.println(gyro.getYComplementaryAngle());
        odometry.update(
            getRotation(), 
            getCurrentModulePositions()
            );
        SmartDashboard.putString("encoder odometry", odometry.getPoseMeters().toString());

        poseEstimator.update(getRotation(), getCurrentModulePositions());

        
        SmartDashboard.putString("pose Estimator", poseEstimator.getEstimatedPosition().toString());


        SmartDashboard.putNumber("Swerve Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Swerve Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Swerve Odosmetry Rot", odometry.getPoseMeters().getRotation().getDegrees());

        // SmartDashboard.putNumber("acc x", gyro2.getYComplementaryAngle());
        // SmartDashboard.putNumber("acc y", gyro2.getYFilteredAccelAngle());
        // SmartDashboard.putNumber("angle", gyro2.getAngle());
        // SmartDashboard.putNumber("acc z", accelerometer.getZ());


        //update the robot pose on the field image on smart dashboard
        field.setRobotPose(poseEstimator.getEstimatedPosition());
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
}