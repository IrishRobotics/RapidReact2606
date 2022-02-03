// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystyem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonSRX Indexer = new WPI_TalonSRX(IndexerConstants.indexerCan);
  
  public IndexerSubsystyem() {
      
  }
  
  public void setSpeed(double speed){
    Indexer.setVoltage(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
