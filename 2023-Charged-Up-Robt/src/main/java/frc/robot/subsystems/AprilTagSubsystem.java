// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagSubsystem extends SubsystemBase {
  // initializing grabbing the data from the camera after processing in photon,
  // name the camera in photon vision the same as the camera name string in code
  PhotonCamera camera = new PhotonCamera("robotcamera");

  
  AprilTag[] tags = (new AprilTag[] {
    new AprilTag(1,
        new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
            new Rotation3d(0, 0, 180))),
    new AprilTag(2,
        new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
            new Rotation3d(0, 0, 180))),
    new AprilTag(3, 
        new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
            new Rotation3d(0, 0, 180))),
    new AprilTag(4, 
        new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
            new Rotation3d(0, 0, 180))),
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
          });

AprilTagFieldLayout layout2023 = new AprilTagFieldLayout(
    List.of(tags),
    Units.inchesToMeters(615.25),
    Units.inchesToMeters(315.5));


  Transform3d cameraPosition = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // initialize the variable for taget ID
  int targetID = -1;

  // height of the camera from the ground
  final double camera_height_in_meters = 0.205;
  // horizontal offset of camera to target (april tag)
  final double camera_pitch_radians = 0;

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(layout2023, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, cameraPosition);
  
  /** Creates a new AprilTagSubsystem. */

  public AprilTagSubsystem() {
    
  }


  @Override
  public void periodic() {

    System.out.println("hi");

    // // update the recieved results from the camera whenever called
    // PhotonPipelineResult cameraResults = camera.getLatestResult();


    // if(cameraResults.hasTargets() == false) {
    //   //Do not update if we do not have targets!!!!
    //   return;
    // }
    
    // PhotonTrackedTarget target = cameraResults.getBestTarget();
    // if(target.getPoseAmbiguity() > 0.2) {
    //   //Do not update if the target is ambiguous
    //   //TODO define this threshold
    //   return;
    // }

    
    // Construct PhotonPoseEstimator

    Optional<EstimatedRobotPose> estPose = photonPoseEstimator.update();


    //TODO MAYBE USE THIS???
    /* 
     var res = cam.getLatestResult();
        if (res.hasTargets()) {
            var imageCaptureTime = res.getTimestampSeconds();
            var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            m_poseEstimator.addVisionMeasurement(
                    camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
        }
    */


    if(estPose.isPresent()) {
      SmartDashboard.putString("pose est", estPose.get().estimatedPose.toString());
    }

    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();

    // // getting the "best" target and identifying its numerical id, printing it for
    // // debug reasons
    // targetID = target.getFiducialId();

    // //Assumes always valid tags AKA the ones on the field
    // Pose3d pose = layout2023.getTagPose(targetID).get();
    // Pose2d targetfield2d = pose.toPose2d();

    // Pose2d target2d = new Pose2d(new Translation2d(bestCameraToTarget.getX(), bestCameraToTarget.getY()), bestCameraToTarget.getRotation().toRotation2d());

    // Translation2d cameraPoseOnField = targetfield2d.getTranslation().plus(target2d.getTranslation());
    
    // SmartDashboard.putNumber("target x position", target2d.getX());
    // SmartDashboard.putNumber("target y position", target2d.getY());
    // SmartDashboard.putNumber("target rotation", target2d.getRotation().getDegrees());

    // SmartDashboard.putString("camera position on field", cameraPoseOnField.toString());

  }

}
