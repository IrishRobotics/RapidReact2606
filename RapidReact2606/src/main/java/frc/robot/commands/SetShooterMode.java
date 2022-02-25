package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.MOTOR_STATUS;

public class SetShooterMode extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem m_subsystem;
    private MOTOR_STATUS setStatus;
    private boolean finished;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetShooterMode(ShooterSubsystem subsystem, ShooterSubsystem.MOTOR_STATUS status) {
      m_subsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
      setStatus = status;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.setMode(setStatus);
        m_subsystem.updateMotorsVel();
        finished=true;
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return finished;
    }
}
