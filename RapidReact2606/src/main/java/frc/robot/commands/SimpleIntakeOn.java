package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.MOTOR_STATUS;

public class SimpleIntakeOn extends CommandBase {
    private IntakeSubsystem m_subsystem;

    public SimpleIntakeOn(IntakeSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_subsystem.setMode(MOTOR_STATUS.ON);
        m_subsystem.updateMotors();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
          m_subsystem.setMode(MOTOR_STATUS.OFF);
          m_subsystem.updateMotors();
      }


    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return true;
      }    
}
