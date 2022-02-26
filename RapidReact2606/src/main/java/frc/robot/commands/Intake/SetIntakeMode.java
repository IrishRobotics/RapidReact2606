package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.MOTOR_STATUS;

public class SetIntakeMode extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_subsystem;
    private MOTOR_STATUS setStatus;
    private boolean finished;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetIntakeMode(IntakeSubsystem subsystem, IntakeSubsystem.MOTOR_STATUS status) {
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
        m_subsystem.updateMotors();
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
