// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax shooter = new CANSparkMax(ShooterConstants.shooterCan, MotorType.kBrushless);
  
  private MODE currentMode = MODE.OFF;

  private MOTOR motor;
  private double curSpeed;

  public enum MODE {
    OFF,SHOOT, COAST
  }
  public enum MOTOR{
    OFF,ON
  }
  public ShooterSubsystem() {
    shooter.setIdleMode(IdleMode.kCoast);
  }

  public void setMode(MODE mode){
    currentMode = mode;
  }
  public MODE getMode(){
    return currentMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentMode) {
      case OFF:
      motor=MOTOR.OFF;
      break;
      case SHOOT:
      motor= MOTOR.ON;
      curSpeed = ShooterConstants.shootSpeed;
      break;
      case COAST:
      motor=MOTOR.ON;
      curSpeed = ShooterConstants.coastSpeed;
      break;
      default:
      motor= MOTOR.OFF;
      break;
    }
    if(motor == MOTOR.OFF){
      shooter.set(0);
    }
    else{
      shooter.set(curSpeed);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
