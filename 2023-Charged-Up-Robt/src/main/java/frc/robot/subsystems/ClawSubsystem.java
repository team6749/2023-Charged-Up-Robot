// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClawSubsystem extends SubsystemBase {
  Compressor compressor = new Compressor(15, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid solenoid = new DoubleSolenoid(15, PneumaticsModuleType.CTREPCM, Constants.Arm.solenoid[0], Constants.Arm.solenoid[1]);
  boolean isClawOpen;
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    compressor.enableDigital();
  }

  public void openSolenoid(){
    solenoid.set(Value.kForward);
    isClawOpen = true;
  }

  public void closeSolenoid(){
    solenoid.set(Value.kReverse);
    isClawOpen = false;
  }

  public void toggleSolenoid(){
    if(isClawOpen == false){
    openSolenoid();
     } else{
      closeSolenoid();
    }
  }

  @Override
  public void periodic() {
    
    if(solenoid.get() != Value.kOff){
      System.err.println("ITS OFF!!!!");
      solenoid.set(Value.kOff);
    }

    // This method will be called once per scheduler run
  }
}
