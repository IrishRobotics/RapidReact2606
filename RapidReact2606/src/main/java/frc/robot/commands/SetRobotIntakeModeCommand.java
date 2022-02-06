package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class SetRobotIntakeModeCommand extends SequentialCommandGroup{
    public SetRobotIntakeModeCommand(IntakeSubsystem intake) {
        addCommands(new SetIntakeMode(intake, IntakeSubsystem.MOTOR_STATUS.ON));
    }
}
