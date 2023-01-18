package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDriveModule {
    // private variables 
    private final WPI_TalonFX driveMotor; 
    private final WPI_TalonFX angleMotor;
    private final WPI_CANCoder absEncoder;
    public final PIDController PIDController = new PIDController(0.01, 0, 0);
    public Translation2d position;
    public double calibrationDegrees;
    public String name;
    //constructor
    /**
     * @param speedMotorId - the port of the speed motor
     * @param angleMotorId - the port of the angle motor
     * @param absEncoderPort - the port of the absolute encoder
     * @param calibrationDegrees - the offset in degrees of the abs encoder
     * @param pos - the Translation2d distance on x/y from the center of the robot
     * Creates a new SwerveModule
     */
    public SwerveDriveModule(String name, int driveMotorID, int angleMotorID, int absEncoderID, double offsetDegrees, Translation2d positionOnRobot){
        this.driveMotor = new WPI_TalonFX(driveMotorID);
        this.angleMotor = new WPI_TalonFX(angleMotorID);
        this.absEncoder = new WPI_CANCoder(absEncoderID);
        this.absEncoder.configMagnetOffset(calibrationDegrees);
        this.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.position = positionOnRobot;
        this.name = name;
        PIDController.enableContinuousInput(0, 360);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setNeutralMode(NeutralMode.Brake);
    }

    //Meters per second
    /**
     * @return the velocity of robot in meters per second
     */
    public double getDriveEncoderVelocity() {
        return driveMotor.getSelectedSensorVelocity() / (2048 * 8.14);
    }

    /**
     * @return the position of the drive motor
     */
    public double getDriveEncoderPos(){
        return Units.metersToFeet(driveMotor.getSelectedSensorPosition() / (2048 * 8.14) * (Math.PI * 0.1));
    }

    //Degrees
    /**
     * @return the rotation of a module in degrees
     */
    public double getRotationEncoder() {
        return (double) Math.round(absEncoder.getAbsolutePosition() * 10000.0) / 10000.0;
    }

    //This must be called all the time
    public void periodic () {
        SmartDashboard.putNumber("Swerve " + name + " rotation", getRotationEncoder());
        SmartDashboard.putNumber("Swerve " + name + " velocity", getDriveEncoderVelocity());
        SmartDashboard.putNumber("Swerve " + name + " position", getDriveEncoderPos());

        printOffsets(calibrationDegrees, getRotationEncoder(), name);
    }

    /**
     * @return returns the state of the SwerveModule in type SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncoderVelocity(), Rotation2d.fromDegrees(getRotationEncoder()));
    }
    /**
     * @param desiredState the desired state of a swerve Module
     * sets the swerve module to the desiredState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        //prevents wheels from resetting back to straight orientation
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getRotationEncoder()));

        SmartDashboard.putNumber("Swerve " + name + " target rotation", state.angle.getDegrees());
    
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
        PIDController.calculate(getRotationEncoder(), state.angle.getDegrees());
        SmartDashboard.putNumber("Swerve " + name + " target pid", turnOutput);

        // // Calculate the drive output from the drive PID controller.
        // final double driveOutput =
        //     _PidController.calculate(getDriveEncoder(), state.speedMetersPerSecond);
    
        // final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
        // final double turnFeedforward =
        //     m_turnFeedforward.calculate(_PidController.getSetpoint().velocity);
    
        // speedMotor.set(driveOutput + driveFeedforward);
        // angleMotor.set(turnOutput + turnFeedforward);

        angleMotor.set(turnOutput);
        //setting the motor to the speed it needs to be speeded
        driveMotor.set(state.speedMetersPerSecond * 0.5);
    }

    /**
     * void function that stops all power to motors
     */
    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void printOffsets(double calibration, double currentAngle, String name){
        //currentAngle includes calibration
        double offset = (double) calibration - currentAngle;
        System.out.println(name + ": " + offset);
    }
}