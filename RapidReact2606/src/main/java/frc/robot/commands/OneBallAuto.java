package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem.MODE;
import frc.robot.subsystems.ShooterSubsystem.MOTOR_STATUS;

public class OneBallAuto extends SequentialCommandGroup{
    public OneBallAuto(ShooterSubsystem shooterSub, IndexerSubsystem indexSub){
        addCommands(new SetShooterMode(shooterSub, MOTOR_STATUS.ON),(new isShooterUpToSpeed(shooterSub)),(new SimpleIndexerOn(indexSub)));
    }
}
