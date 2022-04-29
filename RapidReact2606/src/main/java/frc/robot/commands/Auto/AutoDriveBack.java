package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveBack extends CommandBase{
    public final DriveSubsystem drive;
    private int cnt, end;
    
    public AutoDriveBack(DriveSubsystem robotDrive){
        drive = robotDrive;
        end = (int) Math.round(0.25*(1.0/0.02));
        addRequirements(robotDrive);
    }

    public AutoDriveBack(DriveSubsystem robotDrive, double seconds){
        drive = robotDrive;
        end = (int) Math.round(seconds*(1.0/0.02));
        addRequirements(robotDrive);
    }
    
    @Override
    public void initialize(){
        cnt = 0;
        
    }
    
    //20ms
    @Override
    public void execute(){
        cnt++;
        drive.drive(-0.5,0.0);
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0, 0);
    }
    @Override
    public boolean isFinished(){
        return cnt > end;
    }
}
