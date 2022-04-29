package frc.robot.commands.Auto;

// import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants.PIDConsts.Drive;
import frc.robot.commands.Indexer.SimpleIndexerOn;
import frc.robot.commands.Shooter.SetShooterMode;
import frc.robot.commands.Shooter.isShooterUpToSpeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem.MODE;
import frc.robot.subsystems.ShooterSubsystem.MOTOR_STATUS;

public class AutoWithTargeting extends SequentialCommandGroup{
    public AutoWithTargeting(ShooterSubsystem shooterSub, IndexerSubsystem indexSub, DriveSubsystem drive){
        addCommands(new AutoDriveBack(drive, 0.5), 
        new OneBallAuto(shooterSub, indexSub).withTimeout(8),
        new AutoDriveBack(drive, 1.5)
        );
    }
}
