// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   *
   */
  private static final int shooterTolerance = 50;

  CANSparkMax shooter = new CANSparkMax(ShooterConstants.shooterCan, MotorType.kBrushless);

  RelativeEncoder shootEncoder;
  SparkMaxPIDController pidController;
  public double kP, kI, kD, KIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private PhotonCamera camera;

  public enum MOTOR_STATUS {
    ON, OFF
  };

  MOTOR_STATUS motorState = MOTOR_STATUS.OFF;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(PhotonCamera cam) {
    shooter.setIdleMode(IdleMode.kCoast);
    shootEncoder = shooter.getEncoder();
    pidController = shooter.getPIDController();
    camera = cam;

    // PID coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    KIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    pidController.setP(kP); // position
    pidController.setI(kI); // Integral
    pidController.setD(kD); // Derivitive
    pidController.setIZone(KIz); // INtegral zone
    pidController.setFF(kFF); // feed forward gain value
    pidController.setOutputRange(kMinOutput, kMaxOutput); // output range (in propotion?)
  }

  @Override
  public void periodic() {
    // updateMotorsProp();
    updateMotorsVel();
    SmartDashboard.putNumber("Encoder Vel", shootEncoder.getVelocity());
    SmartDashboard.putBoolean("Is Up to Speed", isUpToSpeed());
  }

  public void updateMotorsProp() {
    if (motorState == MOTOR_STATUS.OFF) {
      shooter.set(0.0);
    } else {
      shooter.set(-0.8);
    }
  }

  public void updateMotorsVel() {
    if (motorState == MOTOR_STATUS.OFF) {
      shooter.set(0.0);
    } else {
      pidController.setReference(getSpeedTarget(), CANSparkMax.ControlType.kVelocity);
    }
  }

  public double getSpeedTarget() {
    PhotonPipelineResult result = camera.getLatestResult();
    double speed = 5000 * -4.284 * ShooterConstants.shootSpeed;

    if (result.hasTargets()) {
      double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.cameraHeight, VisionConstants.targetHeight, 0, Units.degreesToRadians(result.getBestTarget().getPitch()));
      //vary speed based on range
      speed =  5000 * -4.284 * ShooterConstants.shootSpeed;
    } 

    return speed;
  }

  public void setMode(MOTOR_STATUS mode) {
    motorState = mode;
  }

  public boolean isUpToSpeed() {
    return Math.abs(shootEncoder.getVelocity() - getSpeedTarget()) < shooterTolerance;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
