// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autos;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  Thread m_visionThread;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //april tag shit
    m_visionThread =
    new Thread(
        () -> {
          var camera = CameraServer.startAutomaticCapture();

          var cameraWidth = 640;
          var cameraHeight = 480;

          camera.setResolution(cameraWidth, cameraHeight);

          var cvSink = CameraServer.getVideo();
          var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

          var mat = new Mat();
          var grayMat = new Mat();

          var pt0 = new Point();
          var pt1 = new Point();
          var pt2 = new Point();
          var pt3 = new Point();
          var center = new Point();
          var red = new Scalar(0, 0, 255);
          var green = new Scalar(0, 255, 0);

          var aprilTagDetector = new AprilTagDetector();

          var config = aprilTagDetector.getConfig();
          config.quadSigma = 0.8f;
          aprilTagDetector.setConfig(config);

          var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
          quadThreshParams.minClusterPixels = 250;
          quadThreshParams.criticalAngle *= 5; // default is 10
          quadThreshParams.maxLineFitMSE *= 1.5;
          aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

          aprilTagDetector.addFamily("tag16h5");

          var timer = new Timer();
          timer.start();
          var count = 0;

          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
              outputStream.notifyError(cvSink.getError());
              continue;
            }

            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            var results = aprilTagDetector.detect(grayMat);

            var set = new HashSet<>();

            for (var result: results) {
              count += 1;
              pt0.x = result.getCornerX(0);
              pt1.x = result.getCornerX(1);
              pt2.x = result.getCornerX(2);
              pt3.x = result.getCornerX(3);

              pt0.y = result.getCornerY(0);
              pt1.y = result.getCornerY(1);
              pt2.y = result.getCornerY(2);
              pt3.y = result.getCornerY(3);

              center.x = result.getCenterX();
              center.y = result.getCenterY();

              set.add(result.getId());

              Imgproc.line(mat, pt0, pt1, red, 5);
              Imgproc.line(mat, pt1, pt2, red, 5);
              Imgproc.line(mat, pt2, pt3, red, 5);
              Imgproc.line(mat, pt3, pt0, red, 5);

              Imgproc.circle(mat, center, 4, green);
              Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

            };

            for (var id : set){
              System.out.println("Tag: " + String.valueOf(id));
            }

            if (timer.advanceIfElapsed(1.0)){
              System.out.println("detections per second: " + String.valueOf(count));
              count = 0;
            }

            outputStream.putFrame(mat);
          }
          aprilTagDetector.close();
        });
  m_visionThread.setDaemon(true);
  m_visionThread.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    Autos.BlueAutoBalanceOnlyAuto(Constants.DrivebaseConstants.swerveDriveSubsystem).schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
