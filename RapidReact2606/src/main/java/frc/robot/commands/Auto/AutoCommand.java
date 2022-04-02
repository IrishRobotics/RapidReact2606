package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(DriveSubsystem drive, IntakeSubsystem intakeSub) {
        addCommands(
            // Drive forward the specified distance
            new AimToTarget(drive,1),
            new AimToTarget(drive,1.5),
            new TurnToAngle(135, drive),
            new AimToBall(drive,intakeSub),
            new TurnToAngle(0,drive),
            new AimToTarget(drive,1)
        );
    }
}
