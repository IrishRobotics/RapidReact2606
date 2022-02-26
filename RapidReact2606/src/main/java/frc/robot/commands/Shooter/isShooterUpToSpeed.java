package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem.MODE;
import frc.robot.subsystems.IndexerSubsystem.MOTOR;

public class isShooterUpToSpeed extends CommandBase {
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;

    public isShooterUpToSpeed(ShooterSubsystem shooterSub) {
        shooter = shooterSub;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return shooter.isUpToSpeed();
    }
}
