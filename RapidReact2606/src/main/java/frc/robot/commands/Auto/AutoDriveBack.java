package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveBack extends CommandBase{
    public final DriveSubsystem drive;
    private double timeOut, timeToRun;

    public AutoDriveBack(DriveSubsystem robotDrive, double timeToRun){
        drive = robotDrive;
        
        addRequirements(robotDrive);
    }
    
    @Override
    public void initialize(){
        timeOut = System.currentTimeMillis() + timeToRun*1000;
        
    }

    @Override
    public void execute(){
        if(System.currentTimeMillis() > timeOut){
            drive.drive(0.0, 0.0);
        }
        else{
            drive.drive(-1.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0, 0);
    }
    @Override
    public boolean isFinished(){
        return System.currentTimeMillis() > timeOut;
    }
}
