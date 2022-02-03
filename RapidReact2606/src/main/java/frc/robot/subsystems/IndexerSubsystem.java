package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    WPI_TalonSRX indexerWheel = new WPI_TalonSRX(23);

    // 1,0,0 => 0,1,0   1 => 2
    // 1,1,0 => 0,1,1   3 => 6

    DigitalInput intake = new DigitalInput(0);
    DigitalInput ballone = new DigitalInput(1);
    DigitalInput balltwo = new DigitalInput(2);

    private MODE currentMode = MODE.OFF;

    private MOTOR motor;

    public enum MODE {
        OFF, INTAKE, SHOOT
    }

    public enum MOTOR {
        OFF, ON
    }
    

    public void init(){
        indexerWheel.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
        currentMode = MODE.OFF;
    }

    protected int getState()
    {
        int state =0;
        state += intake.get() ? 1 : 0;
        state += (ballone.get() ? 1 : 0) * 2;
        state += (balltwo.get() ? 1 : 0) * 4;

        return state;
    }

    @Override
    public void periodic() {
        switch(currentMode){
            case OFF:
                motor = MOTOR.OFF;
            break;
            case INTAKE:
                motor = getIntakeValue();
            break;
            case SHOOT:
                motor = getShootValue();
            break;
            default:
                motor = MOTOR.OFF;
            break;
        }

        if (motor == MOTOR.ON) {
            indexerWheel.set(1.0);
        } else {
            indexerWheel.set(0);
        }


    }

    private MOTOR getShootValue() {
        return null;
    }

    public MOTOR getIntakeValue(){
        int state = getState();
        MOTOR retVal= motor;

        if ( motor == MOTOR.OFF) {
            if ( state == 1 || state == 3) {
                retVal = MOTOR.ON;
            }
        } else {
            if ( state == 0 || state == 2 || state == 6 || state == 5) {
                retVal = MOTOR.OFF;
            }            
        }

        return retVal;
    }

}

} 