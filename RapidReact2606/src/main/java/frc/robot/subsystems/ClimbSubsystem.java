// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax climbMotor = new CANSparkMax(Constants.ClimbConstants.climbCan, MotorType.kBrushless);

  public enum MOTOR_STATUS {
    ON, OFF
  };
  MOTOR_STATUS mStatus = MOTOR_STATUS.OFF;
  
  public enum MOTOR_DIRECTION{
    OFF, UP, DOWN
  };
  MOTOR_DIRECTION mDirection = MOTOR_DIRECTION.OFF;


  public ClimbSubsystem() {

  }

  @Override
  public void periodic() {
    updateMotors();
  }

  public void updateMotors(){
    if(mStatus == MOTOR_STATUS.ON ){
      switch(mDirection){
        case OFF:
          climbMotor.set(0.0);
        break;
        case UP:
          climbMotor.set(0.85);
        break;
        case DOWN:
          climbMotor.set(-0.85);
        break;
        default:
          climbMotor.set(0.0);
        break;
      }
    }  
    else{
      climbMotor.set(0.0);
    }
  }

  public void setMotorStatus(MOTOR_STATUS ms){
    mStatus = ms;
  }
  public void setMotorDirection(MOTOR_DIRECTION md){
    mDirection = md;
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
