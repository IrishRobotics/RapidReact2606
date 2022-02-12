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

  public enum MOTOR_STATUS {
    ON, OFF
  };

  MOTOR_STATUS motorState = MOTOR_STATUS.OFF;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooter.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    updateMotors();
  }

  public void updateMotors() {
    if (motorState == MOTOR_STATUS.OFF) {
      shooter.set(0.0);
    } else {
      shooter.set(.7);
    }
  }

  public void setMode(MOTOR_STATUS mode) {
    motorState = mode;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
