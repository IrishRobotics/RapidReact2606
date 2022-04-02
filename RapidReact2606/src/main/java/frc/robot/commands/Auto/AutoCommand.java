package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(DriveSubsystem drive) {
        addCommands(
            // Drive forward the specified distance
            new AimToTarget(drive,1),
            new AimToTarget(drive,1.5),
            new TurnToAngle(135, drive),
            new AimToBall(drive),
            new TurnToAngle(0,drive),
            new AimToTarget(drive,1)
        );
    }
}
