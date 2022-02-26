package frc.robot.commands.Combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.SetIndexerMode;
import frc.robot.commands.Intake.SetIntakeMode;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetRobotToIntakeModeCommand extends SequentialCommandGroup {
    public SetRobotToIntakeModeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
        addCommands(new SetIntakeMode(intake, IntakeSubsystem.MOTOR_STATUS.ON),
                new SetIndexerMode(indexer, IndexerSubsystem.MODE.SIMPLE_INTAKE));
    }
}
