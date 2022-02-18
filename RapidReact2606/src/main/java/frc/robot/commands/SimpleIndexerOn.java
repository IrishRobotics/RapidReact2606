package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IndexerSubsystem.MODE;
import frc.robot.subsystems.IndexerSubsystem.MOTOR;

public class SimpleIndexerOn extends CommandBase {
    private IndexerSubsystem m_subsystem;

    public SimpleIndexerOn(IndexerSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setMode(MODE.MANUAL);
        m_subsystem.setMotor(MOTOR.ON);
        m_subsystem.updateMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.updateMotors();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.setMode(MODE.OFF);
        m_subsystem.setMotor(MOTOR.OFF);
        m_subsystem.updateMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
