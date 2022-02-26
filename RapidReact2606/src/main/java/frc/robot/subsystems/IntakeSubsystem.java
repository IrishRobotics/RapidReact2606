package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.intakeCan);

    public enum MOTOR_STATUS {
        ON, OFF
    };

    MOTOR_STATUS motorState = MOTOR_STATUS.OFF;

    public IntakeSubsystem() {
        init();
    }

    public void init() {
        intakeMotor.configFactoryDefault();
    }

    public void setMode(MOTOR_STATUS mode) {
        motorState = mode;
    }

    @Override
    public void periodic() {
        updateMotors();
    }

    public void updateMotors() {
        if (motorState == MOTOR_STATUS.OFF) {
            intakeMotor.set(0.0);
        } else {
            intakeMotor.set(-0.55);
        }
    }
    public void updateMotors(double sped){
        if (motorState == MOTOR_STATUS.OFF) {
            intakeMotor.set(0.0);
        } else {
            intakeMotor.set(-sped);
        }
    }

}
