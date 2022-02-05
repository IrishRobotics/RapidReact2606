package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIndexerMode extends CommandBase {
    private final IndexerSubsystem m_subsystem;
    protected  IndexerSubsystem.MODE currentMode = IndexerSubsystem.MODE.INTAKE;

    public SetIndexerMode(IndexerSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    /**
     * 
     * @param subsystem, Indexer Subsystem
     * @param mode, sets mode OFF, INTAKE, SHOOT; INTAKE default
     */
    public SetIndexerMode(IndexerSubsystem subsystem, IndexerSubsystem.MODE mode) {
        m_subsystem = subsystem;
        if(subsystem.getMode() == mode){
            currentMode = IndexerSubsystem.MODE.OFF;
        }else{
            currentMode = mode;
        }
        addRequirements(subsystem);        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.setMode(currentMode);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }    
}