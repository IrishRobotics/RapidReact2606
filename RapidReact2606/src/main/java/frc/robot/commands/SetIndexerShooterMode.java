package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;

public class SetIndexerShooterMode extends SetIndexerIntakeMode {

    public SetIndexerShooterMode(IndexerSubsystem subsystem) {
        super(subsystem, IndexerSubsystem.MODE.SHOOT);
    }
    
}
