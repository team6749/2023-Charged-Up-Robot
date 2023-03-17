// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClawSubsystem extends SubsystemBase {
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.solenoid[1], Constants.Arm.solenoid[2]);
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
  }

  public void toggleSolenoidState(){
    solenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void closeWrist(double power){
  }
}
