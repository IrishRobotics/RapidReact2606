package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterMode extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    protected ShooterSubsystem.MODE currentMode = ShooterSubsystem.MODE.OFF;

    public SetShooterMode(ShooterSubsystem subsystem){
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    /**
     * 
     * @param subsystem, Shooter Subsystem
     * @param mode, sets mode OFF, COAST, SHOOT; OFF default
     */
    public SetShooterMode(ShooterSubsystem subsystem, ShooterSubsystem.MODE mode){
        m_subsystem = subsystem;
        currentMode = mode;
        
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
